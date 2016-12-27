// Copyright 2016 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

package gpio

import (
	"fmt"
	"log"
	"testing"
	"time"
)

func ExampleAll() {
	fmt.Print("GPIO pins available:\n")
	for _, pin := range All() {
		fmt.Printf("- %s: %s\n", pin, pin.Function())
	}
}

func ExampleByName() {
	p := ByName("GPIO6")
	if p == nil {
		log.Fatal("Failed to find GPIO6")
	}
	fmt.Printf("%s: %s\n", p, p.Function())
}

func ExampleByName_alias() {
	p := ByName("LCD-D2")
	if p == nil {
		log.Fatal("Failed to find LCD-D2")
	}
	if rp, ok := p.(RealPin); ok {
		fmt.Printf("%s is an alias for %s\n", p, rp.Real())
	} else {
		fmt.Printf("%s is not an alias!\n", p)
	}
}

func ExampleByName_number() {
	// The string representation of a number works too.
	p := ByName("6")
	if p == nil {
		log.Fatal("Failed to find GPIO6")
	}
	fmt.Printf("%s: %s\n", p, p.Function())
}

func ExampleByNumber() {
	p := ByNumber(6)
	if p == nil {
		log.Fatal("Failed to find #6")
	}
	fmt.Printf("%s: %s\n", p, p.Function())
}

func ExamplePinIn() {
	p := ByNumber(6)
	if p == nil {
		log.Fatal("Failed to find #6")
	}
	if err := p.In(PullDown, RisingEdge); err != nil {
		log.Fatal(err)
	}
	fmt.Printf("%s is %s\n", p, p.Read())
	for p.WaitForEdge(-1) {
		fmt.Printf("%s went %s\n", p, High)
	}
}

func ExamplePinOut() {
	p := ByNumber(6)
	if p == nil {
		log.Fatal("Failed to find #6")
	}
	if err := p.Out(High); err != nil {
		log.Fatal(err)
	}
}

func ExamplePinStreamReader_ReadStream() {
	// Read one second of sample at 1ms resolution and print the values read.
	b := make(Bits, 1000/8)
	ByNumber(6).(PinStreamReader).ReadStream(PullDown, time.Millisecond, b)
	for i := range b {
		for j := 0; j < 8; j++ {
			fmt.Printf("%s\n", Level(b[i]&(1<<uint(j)) != 0))
		}
	}
}

/*
func ExamplePinStreamReader() {
	// Continuously read samples at 100ms resolution. Create two buffers of 800ms.
	res := 100 * time.Millisecond
	b := []BitStream{{Res: res, Bits: make([]byte, 1)}, {Res: res, Bits: make([]byte, 1)}}
	p := ByNumber(6).(PinStreamReader)
	p.EnqueueReadStream(&b[0])
	for x := 1; ; x = (x + 1) & 1 {
		p.EnqueueReadStream(&b[x])
		// Wait
		for i := range b[x].Bits {
			for j := 7; j >= 0; j-- {
				fmt.Printf("%s\n", Level(b[x].Bits[i]&(1<<uint(j)) != 0))
			}
		}
	}
}
*/

func TestStrings(t *testing.T) {
	if Low.String() != "Low" || High.String() != "High" {
		t.Fail()
	}
	if Float.String() != "Float" || Pull(100).String() != "Pull(100)" {
		t.Fail()
	}
	if NoEdge.String() != "NoEdge" || Edge(100).String() != "Edge(100)" {
		t.Fail()
	}
}

func TestDuty(t *testing.T) {
	data := []struct {
		d        Duty
		expected string
	}{
		{0, "0%"},
		{1, "0%"},
		{DutyMax / 200, "0%"},
		{DutyMax/100 - 1, "1%"},
		{DutyMax / 100, "1%"},
		{DutyMax, "100%"},
		{DutyMax - 1, "100%"},
		{DutyHalf, "50%"},
		{DutyHalf + 1, "50%"},
		{DutyHalf - 1, "50%"},
		{DutyHalf + DutyMax/100, "51%"},
		{DutyHalf - DutyMax/100, "49%"},
	}
	for i, line := range data {
		if actual := line.d.String(); actual != line.expected {
			t.Fatalf("line %d: Duty(%d).String() == %q, expected %q", i, line.d, actual, line.expected)
		}
	}
}

func TestInvalid(t *testing.T) {
	if INVALID.String() != "INVALID" || INVALID.Name() != "INVALID" || INVALID.Number() != -1 || INVALID.Function() != "" {
		t.Fail()
	}
	if INVALID.In(Float, NoEdge) != errInvalidPin || INVALID.Read() != Low || INVALID.WaitForEdge(time.Minute) || INVALID.Pull() != PullNoChange {
		t.Fail()
	}
	if INVALID.Out(Low) != errInvalidPin {
		t.Fail()
	}
}

func TestRegister(t *testing.T) {
	defer reset()
	if err := Register(&basicPin{}, false); err == nil {
		t.Fatal("Expected error")
	}
	if err := Register(&basicPin{N: "a", num: -1}, false); err == nil {
		t.Fatal("Expected error")
	}
	if err := Register(&basicPin{N: "a"}, false); err != nil {
		t.Fatal(err)
	}
	if a := All(); len(a) != 1 {
		t.Fatalf("Expected one pin, got %v", a)
	}
	if a := Aliases(); len(a) != 0 {
		t.Fatalf("Expected zero alias, got %v", a)
	}
	if err := Register(&basicPin{N: "a"}, true); err != nil {
		t.Fatal(err)
	}
	if a := All(); len(a) != 1 {
		t.Fatalf("Expected one pin, got %v", a)
	}
	if a := Aliases(); len(a) != 0 {
		t.Fatalf("Expected zero alias, got %v", a)
	}
	if ByNumber(0) == nil {
		t.Fail()
	}
	if ByNumber(1) != nil {
		t.Fail()
	}
	if ByName("0") == nil {
		t.Fail()
	}
	if ByName("1") != nil {
		t.Fail()
	}
}

func TestRegisterAlias(t *testing.T) {
	defer reset()
	if err := RegisterAlias("", 1); err == nil {
		t.Fatal("Expected error")
	}
	if err := RegisterAlias("alias0", -1); err == nil {
		t.Fatal("Expected error")
	}
	if err := RegisterAlias("alias0", 0); err != nil {
		t.Fatal(err)
	}
	if err := RegisterAlias("alias0", 0); err == nil {
		t.Fatal(err)
	}
	if a := All(); len(a) != 0 {
		t.Fatalf("Expected zero pin, got %v", a)
	}
	if a := Aliases(); len(a) != 0 {
		t.Fatalf("Expected zero alias, got %v", a)
	}
	if err := Register(&basicPin{N: "GPIO0"}, false); err != nil {
		t.Fatal(err)
	}
	if a := All(); len(a) != 1 {
		t.Fatalf("Expected one pin, got %v", a)
	}
	if a := Aliases(); len(a) != 1 {
		t.Fatalf("Expected one alias, got %v", a)
	}
	if p := ByName("alias0"); p == nil {
		t.Fail()
	} else if r := p.(RealPin).Real(); r.Name() != "GPIO0" {
		t.Fatalf("Expected real GPIO0, got %v", r)
	} else if s := p.String(); s != "alias0(GPIO0)" {
		t.Fatal(s)
	}

	if err := Register(&basicPin{N: "GPIO1"}, false); err == nil {
		t.Fail()
	}
	if err := Register(&basicPin{N: "GPIO0", num: 1}, false); err == nil {
		t.Fail()
	}
	if err := Register(&basicPin{N: "alias0", num: 1}, false); err == nil {
		t.Fail()
	}
	if err := Register(&pinAlias{&basicPin{N: "GPIO1", num: 1}, "alias1", 2}, false); err == nil {
		t.Fail()
	}
}

func TestAreInGPIOTest(t *testing.T) {
	// Real tests are in gpiotest due to cyclic dependency.
}

//

// basicPin implements Pin as a non-functional pin.
type basicPin struct {
	invalidPin
	N   string
	num int
}

func (b *basicPin) String() string {
	return b.N
}

func (b *basicPin) Name() string {
	return b.N
}

func (b *basicPin) Number() int {
	return b.num
}

func reset() {
	mu.Lock()
	defer mu.Unlock()
	byNumber = [2]map[int]PinIO{map[int]PinIO{}, map[int]PinIO{}}
	byName = [2]map[string]PinIO{map[string]PinIO{}, map[string]PinIO{}}
	byAlias = map[string]*pinAlias{}
}
