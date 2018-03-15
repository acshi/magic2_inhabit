import os
import server

os.chdir(os.path.split(server.__file__)[0] + '/site')
server.run()
