import http.server
import socketserver

port=8000

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

print("Запуск бляяя")
print("http:localhost:8000")
print("CANCEL CTRL+C")

with socketserver.TCPServer(("", port), MyHandler) as httpd:
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n СТОЯТЬ")