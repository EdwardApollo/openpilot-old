#!/usr/bin/env python3
from flask import Flask
import cereal.messaging as messaging

app = Flask(__name__)
pm = messaging.PubMaster(['testJoystick'])

@app.route("/")
def hello_world():
  return open("index.html").read()

@app.route("/control/<x>/<y>")
def control(x, y):
  x,y = int(x), int(y)
  dat = messaging.new_message('testJoystick')
  dat.testJoystick.axes = [x,y]
  pm.send('testJoystick', dat)
  return ""

if __name__ == '__main__':
  app.run(host="0.0.0.0", debug=True)

