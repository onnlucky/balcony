var client

f = io.File("data.txt").open(create=true)
s = io.Socket.listen(0xda1a >> 3)
s2 = io.Socket.listen(8080)

log.info("data logger running:", s.ip, s.port)

!s.serve(conn -> !(
    log.info("client", conn.ip, conn.port)
    client = conn

    catch: e ->
        client = null
        log.error(e)

    log.debug("sending time", localtime())
    conn.write((time() + localtime().gmtoff).round.toString, "\n")
    loop:
        at = conn.find("\n")
        if not at: break
        data = conn.readString(at).eval
        if data:
            data = {time=time().round, ip=conn.ip, port=conn.port, data=data}
            log.debug(data)
            f.write(repr(data), "\n")
))

!s2.serve(conn ->
    catch: e ->
        log.error(e)
        conn.write("fail\n")
    cmd = conn.readLine()
    print "command: '$(repr(cmd))'"
    if not cmd: break
    if not client:
        conn.write("fail: no client\n")
        break
    client.write(cmd, "\n")
    conn.write("ok\n")
)

