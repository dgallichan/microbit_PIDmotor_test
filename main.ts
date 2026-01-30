function updatePID (mx: number, my: number) {
    // 1. Complex multiplication to find the rotational difference
    // This naturally handles the 360 wrap-around
    dot = targetX * mx + targetY * my
    cross = targetY * mx - targetX * my
    // 2. The error is the angle of that difference
    error = Math.atan2(cross, dot) * (180 / Math.PI)
    // error = target - current
    // 1. Proportional: Direct response to error
    P = kp * error
    // 2. Integral: Fixes long-term drift (with clamping to prevent "windup")
    integral = integral + error
    if (integral > 100) {
        integral = 100
    }
    if (integral < -100) {
        integral = -100
    }
    I = ki * integral
    // 3. Derivative: Dampens "bouncing"
    derivative = error - lastError
    D = kd * derivative
    lastError = error
    output = P + I + D
    return output
}
let speed = 0
let rotAngle = 0
let yComp = 0
let xComp = 0
let magAbs = 0
let output = 0
let D = 0
let lastError = 0
let derivative = 0
let I = 0
let integral = 0
let P = 0
let error = 0
let cross = 0
let dot = 0
let kd = 0
let ki = 0
let kp = 0
let targetY = 0
let targetX = 0
let targetAngle = -110
targetX = Math.cos(targetAngle * Math.PI / 180)
targetY = Math.sin(targetAngle * Math.PI / 180)
basic.showIcon(IconNames.Heart)
kp = 0.1
ki = 0.001
kd = 0.5
control.inBackground(function () {
    while (true) {
        magAbs = input.magneticForce(Dimension.Strength)
        xComp = input.magneticForce(Dimension.X) / magAbs
        yComp = input.magneticForce(Dimension.Y) / magAbs
        rotAngle = Math.atan2(yComp, xComp)
        if (input.buttonIsPressed(Button.A)) {
            speed = updatePID(xComp, yComp)
            if (speed > 0) {
                Kitronik_Robotics_Board.motorOn(Kitronik_Robotics_Board.Motors.Motor1, Kitronik_Robotics_Board.MotorDirection.Reverse, Math.max(1, Math.abs(speed)))
            } else {
                Kitronik_Robotics_Board.motorOn(Kitronik_Robotics_Board.Motors.Motor1, Kitronik_Robotics_Board.MotorDirection.Forward, Math.max(1, Math.abs(speed)))
            }
        } else {
            Kitronik_Robotics_Board.motorOff(Kitronik_Robotics_Board.Motors.Motor1)
        }
        serial.writeValue("P", P)
        serial.writeValue("I", I)
        serial.writeValue("D", D)
        serial.writeValue("c", rotAngle)
        serial.writeValue("speed", speed)
        control.waitMicros(1000)
    }
})
