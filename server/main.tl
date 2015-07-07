influxPoint = conn, d, line ->
    conn = line or conn
    d = line?data or d

    body = "balcony,r=$(d.r)" +
        " ip=\"$(conn.ip)\",port=$(conn.port),uptime=$(d.uptime)" +
        ",pump_on=$(d.pump.on > 0),pump_duration=$(d.pump.duration),pump_next=$(d.pump.next),pump_last=$(d.pump.last)" +
        ",battery_level=$(d.battery.level)" +
        ",mainbucket_level=$(d.mainbucket.level)" +
        ",highbucket_level=$(d.highbucket.level)" +
        ",celcius=$(d.temperature.celcius),humidity=$(d.temperature.humidity)" +
        if(line, " $(line.time)\n", "\n")

    body, res = http.post("http://localhost:8086/write?db=balcony&precision=s", body)
    if res.status < 200 and res.status >= 300: log.warning(res.status, body)

var client
var last_data

s = io.Socket.listen(0xda1a >> 3)
server = http.Server.new(8080)

log.info("data logger running:", s.ip, s.port)

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
        d = conn.readString(at).eval
        last_data = d
        influxPoint(conn, d)

    client = null
    log.info("client closed")
))

server.serve(conn ->
    method, path = multi(conn.req.method, conn.req.url.path)
    log.info("web client", conn.ip, conn.port, method, path)

    if method == "GET" and path == "/status":
        conn.setContentType("application/json")
        if client:
            conn.write json.toString(last_data)
            return
        conn.write "{}"
        return
    if method == "GET" and path == "/":
        conn.setContentType("text/html")
        conn.write(io.File("index.html").read)
        return

    conn.status = 404
    conn.write("404 Not Found")
)
