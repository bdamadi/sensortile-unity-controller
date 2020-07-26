const gatt = require('./gatt')
const WebSocket = require('ws')
const wss = new WebSocket.Server({ port: 1234 })

let gattProcess = null

wss.on('listening', () => console.log('Start listening...'))

wss.on('connection', (ws) => {
  console.log(`${ws._socket.remoteAddress}: Connected!`)

  // Receive command from websocket client, e.i. Unity
  ws.on('message', (message) => {
    console.log(`${ws._socket.remoteAddress}: << ${message}`)
    if (message === 'GATT START') {
      if (gattProcess == null) {
        gattProcess = gatt(({ value }) => {
          console.log(`${ws._socket.remoteAddress}: >> ${value}`)
          ws.send(value)
        })
      } else {
        console.log('gatttool is already running')
      }
    } else if (message === 'GATT STOP') {
      if (gattProcess != null) {
        gattProcess.kill()
        gattProcess = null
      } else {
        console.log('gatttool is not running')
      }
    }
  })

  ws.on('close', () => {
    if (gattProcess != null) {
      gattProcess.kill()
      gattProcess = null
    }
    console.log(`${ws._socket.remoteAddress}: Closed!`)
  })

  ws.send('READY')
})
