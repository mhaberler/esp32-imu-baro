#!/usr/bin/env python2

""" daemon implementing zeroconf publishing service for chains master servers"""
from __future__ import absolute_import, division, print_function, unicode_literals

import socket
from zeroconf import ServiceInfo, Zeroconf

# Taken from: http://stackoverflow.com/a/11735897
def get_local_ip() -> str:
    """Try to determine the local IP address of the machine."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Use Google Public DNS server to determine own IP
        sock.connect(("8.8.8.8", 80))

        return sock.getsockname()[0]  # type: ignore
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except socket.gaierror:
            return "127.0.0.1"
    finally:
        sock.close()

def register(port, desc, servername, mdns):
    zeroconf = Zeroconf()
    host_ip = get_local_ip()

    try:
        host_ip_pton = socket.inet_pton(socket.AF_INET, host_ip)
    except OSError:
        host_ip_pton = socket.inet_pton(socket.AF_INET6, host_ip)
    
    hostname = socket.gethostname()
    services = []
    desc = {'Description': 'Chains Home Automation service on rabbitmq'}
    prefix = "_http._tcp.local."
    info = ServiceInfo(
        prefix,
        mdns + prefix,
        addresses=[host_ip_pton],
        port=80,
        properties={"foo":"bar"},
        server="ash-2.local.",
    )
    services.append(info)
    for info in services:
        zeroconf.register_service(info)
    return services

    # try:
    #     input("Waiting (press Enter to exit)...")
    # finally:
    #     print("Unregistering...")
    #     zeroconf.unregister_service(info)
    #     zeroconf.close()


if __name__ == '__main__':
    print('Starting zeroconf publishing service')
    main()

