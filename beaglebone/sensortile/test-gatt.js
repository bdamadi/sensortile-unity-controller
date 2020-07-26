const gatt = require('./gatt')

const child = gatt()
setTimeout(() => {
  console.log('Kill...')
  child.kill()
}, 5000)
