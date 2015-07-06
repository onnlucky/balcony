var client
data = Array.new
max = 60 / 5 * 60 * 24 * 2 # days

f = io.File("data.txt").open(create=true)
loop:
    line = f.readLine()
    if not line: break
    line = line.eval
    data.add(line)

s = io.Socket.listen(0xda1a >> 3)
http = http.Server.new(8080)

log.info("data logger running:", s.ip, s.port, "read from disk:", data.size, "logs")

!s.serve(conn -> !(
    log.info("client", conn.ip, conn.port)
    client = conn

    catch: e ->
        client = null
        log.error(e)

    conn.write((time() + localtime().gmtoff).round.toString, "\n")
    loop:
        at = conn.find("\n")
        if not at: break
        line = conn.readString(at).eval
        if line:
            line = {time=time().round, ip=conn.ip, port=conn.port, data=line}
            log.debug(line)
            f.write(repr(line), "\n")
            data.add(line)
            while data.size > max: data.remove(1)

    client = null
    log.info("client closed")
))

http.serve(conn ->
    method, path = multi(conn.req.method, conn.req.url.path)
    log.info("web client", conn.ip, conn.port, method, path)

    if method == "GET" and path == "/data":
        conn.setContentType("application/json")
        from = Number(conn.req.url.query["from"]) or 0
        tosend = { from == 0 }: data
                 {           }: at = data.find(d -> d.time == from) or 0
                                data[at + 1:]
        conn.write(json.toString({client=bool(client),data=tosend}))
        return

    if method == "GET" and path == "/":
        conn.setContentType("text/html")
        conn.write(io.File("index.html").read)
        return

    conn.status = 404
    conn.write("404 Not Found")
)

