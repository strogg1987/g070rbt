package main

import (
	"time"

	"github.com/goburrow/modbus"
	"github.com/sirupsen/logrus"
)

func main() {
	logrus.SetFormatter(&logrus.TextFormatter{
		FullTimestamp:   true,
		TimestampFormat: "2006-01-02 15:04:05",
	})
	logrus.Println("start")
	// Modbus RTU/ASCII
	handler := modbus.NewRTUClientHandler("/dev/ttyACM0")
	handler.BaudRate = 19200
	handler.DataBits = 8
	handler.Parity = "N"
	handler.StopBits = 1
	handler.SlaveId = 1
	handler.Timeout = 5 * time.Second
	handler.SlaveId = 1

	err := handler.Connect()
	if err != nil {
		logrus.Fatal(err)
	}
	defer handler.Close()

	client := modbus.NewClient(handler)
	results, err := client.ReadCoils(1, 16)
	if err != nil {
		logrus.Fatal(err)
	}

	logrus.Printf("COILS:\t%08b %08b", results[1], results[0])
	results, err = client.WriteMultipleCoils(1, 2, []byte{0x00})
	if err != nil {
		logrus.Fatal("write :", err)
	}

	logrus.Printf("RESP: \t%08b %08b", results[0], results[1])
	results, err = client.ReadCoils(1, 16)
	if err != nil {
		logrus.Fatal(err)
	}

	logrus.Printf("COILS:\t%08b %08b", results[1], results[0])
}
