const { spawn } = require('child_process')

const MAC_ADDRESS_SENSORTILE = 'C0:83:3B:39:5D:30'

const GATT_TOOL = 'gatttool'
const GATT_ARGS = [
  '-b', MAC_ADDRESS_SENSORTILE,
  '-t', 'random',
  '--char-write-req',
  '--handle=0x0012',
  '--value=0100',
  '--listen'
]

const DATA_EXP = /(?:handle\s+=\s+)(0x[0-9a-f]{4})\s+(?:value:\s+)([0-9a-f]{2}(?:\s*[0-9a-f]{2})*)/i

function listen (onDataCallback) {
  console.log(`Starting ${GATT_TOOL}...`)
  const child = spawn(GATT_TOOL, GATT_ARGS)
  // const child = spawn('tail', ['-f', 'sample.txt'])

  // Error when the child process could not start
  child.on('error', (err) => {
    console.error(`Failed to start ${GATT_TOOL}:`, err)
  })

  // Read output from the child process
  child.stdout.on('data', (data) => {
    const dataStr = data.toString()
    console.log('Output:', dataStr)

    dataStr.split('\n').forEach(str => {
      const matches = str.match(DATA_EXP)
      if (matches) {
        onDataCallback({
          handle: matches[1],
          value: matches[2]
        })
      }
    })
  })

  // When some error is written
  child.stderr.on('data', (data) => {
    console.error('Error:', data.toString())
  })

  // When the solver script finishes
  child.on('close', (code, signal) => {
    console.log(`${GATT_TOOL} process exited with code ${code} by signal ${signal}`)
  })

  return child
}

module.exports = listen
