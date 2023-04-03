const { SerialPort } = require('serialport')

async function Main() {
  if (process.argv.length > 2) {
    const port = new SerialPort({
      path: process.argv[2],
      baudRate: 1200,
    }).on('error', (err) => {
    })
  }
  else {
    console.log("No serial port specified! - Usage: node index.js <serial port>")
  }
}

Main()
