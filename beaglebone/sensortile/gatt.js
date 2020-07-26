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

function listen () {
  console.log(`Starting ${GATT_TOOL}...`)
  const child = spawn(GATT_TOOL, GATT_ARGS)
  // const child = spawn('tail', ['-f', 'sample.txt'])

  // Error when the child process could not start
  child.on('error', (err) => {
    console.error(`Failed to start ${GATT_TOOL}:`, err)
  })

  // Read output from the child process
  child.stdout.on('data', (data) => {
    console.log('Output:', data.toString())
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
