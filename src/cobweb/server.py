import BaseHTTPServer
import SocketServer
import csv
from goal_request_t import goal_request_t
import lcm
import signal
import re


PORT = 8206


class CobwebRequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        self.lcm = lcm.LCM()
        BaseHTTPServer.BaseHTTPRequestHandler.__init__(self, *args, **kwargs)

    def _set_headers(self, code):
        self.send_response(code)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        self._set_headers(200)
        self.wfile.write(HTML_STRING)

    def do_POST(self):
        self._set_headers(204)
        data = self.rfile.read(int(self.headers['Content-Length']))
        print data
        goal = data.split('=')[-1]

        # TODO: Communicate the goal more reliably.
        msg = goal_request_t()
        msg.x = GOALS[goal][0]
        msg.y = GOALS[goal][1]
        self.lcm.publish("GOAL_REQUEST", msg.encode())


def run():
    global HTML_STRING, GOALS
    # Do some adhoc templating.
    page_template = open('index.html').read()
    item_template_regex = '{#(.*)#}'
    item_template = re.search(item_template_regex, page_template, re.S).group(1)
    items_string = ''

    GOALS = {}
    for title, x, y in csv.reader(open('../goals.csv')):
        sanitized_title = title.replace('\'','').replace(' ', '_')
        GOALS[sanitized_title] = (float(x), float(y), title)

        items_string += item_template \
                .replace('{{goal_value}}', sanitized_title) \
                .replace('{{goal_title}}', title)

    HTML_STRING = re.sub(item_template_regex, items_string, page_template, 0, re.S)

    # Create server.
    Handler = CobwebRequestHandler
    httpd = BaseHTTPServer.HTTPServer(("", PORT), Handler)

    httpd.serve_forever()
