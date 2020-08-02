const { spawn } = require('child_process')

const MAC_ADDRESS_SENSORTILE = 'AA:AA:AA:DD:EE:FF'

const GATT_TOOL = 'gatttool'
const GATT_ARGS = [
  '-b', MAC_ADDRESS_SENSORTILE,
  // '-t', 'random', // Not use for static MAC address
  '--listen'
]

const DATA_EXP = /(?:handle\s+=\s+)(0x[0-9a-f]{4})\s+(?:value:\s+)([0-9a-f]{2}(?:\s*[0-9a-f]{2})*)/i

function writeRequest (handle, value, { onData, onError, onComplete }) {
  console.log(`Starting ${GATT_TOOL}...`)
  const child = spawn(GATT_TOOL, [
    ...GATT_ARGS,
    '--char-write-req',
    `--handle=0x${handle}`,
    `--value=${value}`
  ])
  // const child = spawn('tail', ['-f', 'sample.txt'])

  // Error when the child process could not start
  child.on('error', (err) => {
    console.error(`Failed to start ${GATT_TOOL}:`, err)
    onError && onError(err)
  })

  let recvBuffer = ''

  // Read output from the child process
  child.stdout.on('data', (data) => {
    const dataStr = data.toString()
    console.log('Output:', dataStr)

    recvBuffer += dataStr
    console.log('Recv Buffer')
    console.log(recvBuffer)
    console.log('Recv Buffer -- End')

    const lines = recvBuffer.split('\n')
    recvBuffer = ''
    lines.forEach((str, index) => {
      const matches = str.match(DATA_EXP)
      if (matches) {
        onData && onData({
          handle: matches[1],
          value: matches[2]
        })
      } else if (index === lines.length - 1) {
        recvBuffer = str + '\n'
      }
    })
  })

  // When some error is written
  child.stderr.on('data', (data) => {
    const err = data.toString()
    console.error('Error:', err)
    onError && onError(err)
  })

  // When the solver script finishes
  child.on('close', (code, signal) => {
    console.log(`${GATT_TOOL} process exited with code ${code} by signal ${signal}`)
    onComplete && onComplete()
  })

  return child
}

module.exports = {
  writeRequest
}
