const { writeRequest } = require('./gatt')
const WebSocket = require('ws')
const wss = new WebSocket.Server({ port: 1234 })

const GATT_WRITE_CMD = /GATT WRITE ([0-9a-f]{4}) ([0-9a-f]{4})/i

let gattProcess = null

wss.on('listening', () => console.log('Start listening...'))

wss.on('connection', (ws) => {
  console.log(`${ws._socket.remoteAddress}: Connected!`)

  const onData = ({ value }) => {
    console.log(`${ws._socket.remoteAddress}: >> DATA:${value}`)
    ws.send('DATA:' + value)
  }
  const onError = (err) => {
    console.log(`${ws._socket.remoteAddress}: >> ERROR:${err}`)
    ws.send('ERROR:' + err)
  }

  // Receive command from websocket client, e.i. Unity
  ws.on('message', (message) => {
    console.log(`${ws._socket.remoteAddress}: << ${message}`)

    const matches = message.match(GATT_WRITE_CMD)
    if (matches) {
      if (gattProcess == null) {
        const handle = matches[1]
        const value = matches[2]
        gattProcess = writeRequest(handle, value, { onData, onError })
      } else {
        console.log('gatttool is already running')
        ws.send(`ERROR:gatttool is already running`)
      }
    } else if (message === 'GATT STOP') {
      if (gattProcess != null) {
        gattProcess.kill()
        gattProcess = null
      } else {
        console.log('gatttool is not running')
        ws.send(`ERROR:gatttool is not running`)
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
