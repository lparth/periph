// Copyright 2016 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

package allwinner

import (
	"time"

	"github.com/google/periph/conn/gpio"
)

func ExamplePin_PWM() {
	if p, ok := gpio.ByName("PWM0").(*Pin); ok {
		p.PWM(32536, 10*time.Microsecond)
	}
}
