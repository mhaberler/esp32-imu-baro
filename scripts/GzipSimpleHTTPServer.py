"""Simple HTTP Server.

This module builds on HTTPServer by implementing the standard GET
and HEAD requests in a fairly straightforward manner.

"""


__version__ = "0.6"

__all__ = ["SimpleHTTPRequestHandler"]

import os
import posixpath
from http.server import HTTPServer as BaseHTTPServer
from http.server import SimpleHTTPRequestHandler
import urllib
import cgi
import sys
import mimetypes
import zlib
import brotli
#import urllib.parse
from optparse import OptionParser
from io import StringIO
import html
import socket
from zeroconf import ServiceInfo, Zeroconf

# import announce
# import zeroconf

#try:
#    from cStringIO import StringIO
#except ImportError:
#    from StringIO import StringIO

SERVER_PORT = 8000
encoding_type = 'gzip'

def parse_options():
    # Option parsing logic.
    parser = OptionParser()
    parser.add_option("-e", "--encoding", dest="encoding_type",
                      help="Encoding type for server to utilize",
                      metavar="ENCODING", default='gzip')
    global SERVER_PORT
    parser.add_option("-p", "--port", dest="port", default=SERVER_PORT,
                      type="int",
                      help="The port to serve the files on",
                      metavar="ENCODING")
    parser.add_option("-w", "--web", dest="web",
                      help="The directory to serve files from")
    parser.add_option("-m", "--mdns", dest="mdns",
                      help="announce via MDNS")
    parser.add_option("-t", "--trace", dest="trace",
                      help="trace arguments and execution")
    
    (options, args) = parser.parse_args()

    print(f"----webserver got: {options=}  {args=}")

    global encoding_type
    encoding_type = options.encoding_type
    SERVER_PORT = int(options.port)

    if encoding_type not in ['zlib', 'deflate', 'gzip', 'br']:
        sys.stderr.write("Please provide a valid encoding_type for the server to utilize.\n")
        sys.stderr.write("Possible values are 'zlib', 'gzip', 'br' and 'deflate'\n")
        sys.stderr.write("Usage: python GzipSimpleHTTPServer.py --encoding=<encoding_type>\n")
        sys.exit()

    if options.web:
        os.chdir(options.web)

    return (options, args)

def zlib_encode(content):
    zlib_compress = zlib.compressobj(9, zlib.DEFLATED, zlib.MAX_WBITS)
    data = zlib_compress.compress(content) + zlib_compress.flush()
    return data


def deflate_encode(content):
    deflate_compress = zlib.compressobj(9, zlib.DEFLATED, -zlib.MAX_WBITS)
    data = deflate_compress.compress(content) + deflate_compress.flush()
    return data


def gzip_encode(content):
    gzip_compress = zlib.compressobj(9, zlib.DEFLATED, zlib.MAX_WBITS | 16)
    data = gzip_compress.compress(content) + gzip_compress.flush()
    return data

def br_encode(content):
    data = brotli.compress(content)
    return data

class SimpleHTTPRequestHandler(SimpleHTTPRequestHandler):
    """Simple HTTP request handler with GET and HEAD commands.

    This serves files from the current directory and any of its
    subdirectories.  The MIME type for files is determined by
    calling the .guess_type() method.

    The GET and HEAD requests are identical except that the HEAD
    request omits the actual contents of the file.

    """

    server_version = "SimpleHTTP/" + __version__

    def do_GET(self):
        """Serve a GET request."""
        content = self.send_head()
        if content:
            if type(content) is str:
                content=content.encode("utf-8")
            self.wfile.write(content)

    def do_HEAD(self):
        """Serve a HEAD request."""
        content = self.send_head()

    def send_head(self):
        """Common code for GET and HEAD commands.

        This sends the response code and MIME headers.

        Return value is either a file object (which has to be copied
        to the outputfile by the caller unless the command was HEAD,
        and must be closed by the caller under all circumstances), or
        None, in which case the caller has nothing further to do.

        """
        path = self.translate_path(self.path)
        print("Serving path '%s'" % path)
        f = None
        if os.path.isdir(path):
            if not self.path.endswith('/'):
                # redirect browser - doing basically what apache does
                self.send_response(301)
                self.send_header("Location", self.path + "/")
                self.end_headers()
                return None
            for index in "index.html", "index.htm":
                index = os.path.join(path, index)
                if os.path.exists(index):
                    path = index
                    break
            else:
                return self.list_directory(path).read()
        ctype = self.guess_type(path)
        staticcf=False
        try:
            # Always read in binary mode. Opening files in text mode may cause
            # newline translations, making the actual size of the content
            # transmitted *less* than the content-length!
            if os.path.exists(path+".gz"):
                staticcf=True
                f=open(path+".gz",'rb')
            else:
                f = open(path, 'rb')
        except IOError:
            self.send_error(404, "File not found")
            return None
        self.send_response(200)
        self.send_header("Content-type", ctype)
        self.send_header("Content-Encoding", encoding_type)
        fs = os.fstat(f.fileno())
        raw_content_length = fs[6]
        content = f.read()

        # Encode content based on runtime arg
        if not staticcf:
            if encoding_type == "gzip":
                content = gzip_encode(content)
            elif encoding_type == "deflate":
                content = deflate_encode(content)
            elif encoding_type == "zlib":
                content = zlib_encode(content)
            elif encoding_type == "br":
                content = br_encode(content)

        compressed_content_length = len(content)
        f.close()
        self.send_header("Content-Length", max(raw_content_length, compressed_content_length))
        self.send_header("Last-Modified", self.date_time_string(fs.st_mtime))
        self.end_headers()
        return content

    def list_directory(self, path):
        """Helper to produce a directory listing (absent index.html).

        Return value is either a file object, or None (indicating an
        error).  In either case, the headers are sent, making the
        interface the same as for send_head().

        """
        try:
            list = os.listdir(path)
        except os.error:
            self.send_error(404, "No permission to list directory")
            return None
        list.sort(key=lambda a: a.lower())
        f = StringIO()
        displaypath = html.escape(urllib.parse.unquote(self.path))
        f.write('<!DOCTYPE html>')
        f.write("<html>\n<title>Directory listing for %s</title>\n" % displaypath)
        f.write("<body>\n<h2>Directory listing for %s</h2>\n" % displaypath)
        f.write("<hr>\n<ul>\n")
        for name in list:
            fullname = os.path.join(path, name)
            displayname = linkname = name
            # Append / for directories or @ for symbolic links
            if os.path.isdir(fullname):
                displayname = name + "/"
                linkname = name + "/"
            if os.path.islink(fullname):
                displayname = name + "@"
                # Note: a link to a directory displays with @ and links with /
            f.write('<li><a href="%s">%s</a>\n'
                    % (urllib.parse.quote(linkname), html.escape(displayname)))
        f.write("</ul>\n<hr>\n</body>\n</html>\n")
        length = f.tell()
        f.seek(0)
        self.send_response(200)
        encoding = sys.getfilesystemencoding()
        self.send_header("Content-type", "text/html; charset=%s" % encoding)
        self.send_header("Content-Length", str(length))
        self.end_headers()
        return f

    def translate_path(self, path):
        """Translate a /-separated PATH to the local filename syntax.

        Components that mean special things to the local file system
        (e.g. drive or directory names) are ignored.  (XXX They should
        probably be diagnosed.)

        """
        # abandon query parameters
        path = path.split('?',1)[0]
        path = path.split('#',1)[0]
        path = posixpath.normpath(urllib.parse.unquote(path))
        words = path.split('/')
        words = filter(None, words)
        path = os.getcwd()
        for word in words:
            drive, word = os.path.splitdrive(word)
            head, word = os.path.split(word)
            if word in (os.curdir, os.pardir): continue
            path = os.path.join(path, word)
        return path

    def guess_type(self, path):
        """Guess the type of a file.

        Argument is a PATH (a filename).

        Return value is a string of the form type/subtype,
        usable for a MIME Content-type header.

        The default implementation looks the file's extension
        up in the table self.extensions_map, using application/octet-stream
        as a default; however it would be permissible (if
        slow) to look inside the data to make a better guess.

        """

        base, ext = posixpath.splitext(path)
        if ext in self.extensions_map:
            return self.extensions_map[ext]
        ext = ext.lower()
        if ext in self.extensions_map:
            return self.extensions_map[ext]
        else:
            return self.extensions_map['']

    if not mimetypes.inited:
        mimetypes.init() # try to read system mime.types
    extensions_map = mimetypes.types_map.copy()
    extensions_map.update({
        '': 'application/octet-stream', # Default
        '.py': 'text/plain',
        '.c': 'text/plain',
        '.h': 'text/plain',
        })

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

def register( port, desc, servername, mdns):
    zeroconf = Zeroconf()
    host_ip = get_local_ip()

    try:
        host_ip_pton = socket.inet_pton(socket.AF_INET, host_ip)
    except OSError:
        host_ip_pton = socket.inet_pton(socket.AF_INET6, host_ip)
    
    hostname = socket.gethostname()
    services = []
    type_ = "_http._tcp.local."
        # "_http._tcp.local.",
        # "Paul's Test Web Site._http._tcp.local.",
    info = ServiceInfo(
        type_,
        mdns + "." + type_,
        addresses=[host_ip_pton],
        port=port,
        properties=desc,
        server=hostname,
    )
    services.append(info)
    for info in services:
        zeroconf.register_service(info)
    return zeroconf, services

def unregister(zeroconf, services):
    print(f"Unregistering {services=} ...")
    for info in services:
        zeroconf.unregister_service(info)
    zeroconf.close()
    print(f"done.")


def main(HandlerClass = SimpleHTTPRequestHandler,
         ServerClass = BaseHTTPServer):
    """Run the HTTP request handler class.

    This runs an HTTP server on port 8000 (or the first command line
    argument).

    """
    (options, args) = parse_options()

    server_address = ('0.0.0.0', SERVER_PORT)

    SimpleHTTPRequestHandler.protocol_version = "HTTP/1.0"
    httpd = BaseHTTPServer(server_address, SimpleHTTPRequestHandler)

    sa = httpd.socket.getsockname()
    #print "Serving HTTP on", sa[0], "port", sa[1], "..."

    if options.mdns:
        zeroconf, services = register(options.port, {"foo":"bar"}, "foobar.local", options.mdns)

    try:
        print("Serving HTTP on", sa[0], "port", sa[1], "...")
        httpd.serve_forever()
    except KeyboardInterrupt:
        if options.mdns:
            unregister(zeroconf, services)



if __name__ == '__main__':
    main()
