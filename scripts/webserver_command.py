Import("env")


# see https://docs.platformio.org/en/latest/scripting/examples/platformio_ini_custom_options.html
config = env.GetProjectConfig()
port = config.get("http_server", "port")
mdns = config.get("http_server", "mdns")
root = config.get("http_server", "root")
trace = config.get("http_server", "trace")
encoding = config.get("http_server", "encoding")

print(f" --- httpserver options: {port=} {mdns=} {root=} {encoding=} {trace=}")

# Multiple actions
env.AddCustomTarget(
    name="http_server",
    dependencies=None,
    actions=[
        f"python scripts/GzipSimpleHTTPServer.py --web {root} "
        f"--port {port} "
        f"--encoding {encoding} "
        f"--mdns {mdns}"
    ],
    title="HTTP Server",
    description="Run local HTTP Server"
)
