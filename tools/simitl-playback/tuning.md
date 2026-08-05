
# propeller inertia

If you do the real-world physics math for a 5" quadcopter prop, `6.9e-06` kg·m² is almost exactly the correct moment of inertia. Here is why:

### The Math:
You can roughly model a drone propeller as a thin rod spinning around its center. The formula for the moment of inertia of a rod is:
**$I = \frac{1}{12} m L^2$**

*   **Length ($L$):** 5 inches = **0.127 meters**
*   **Mass ($m$):** A typical 5" polycarbonate prop (like an HQProp or Gemfan) weighs about 4.5 grams = **0.0045 kg**

If we plug those in:

$I = \frac{1}{12} \times 0.0045 \times (0.127)^2$

$I = 0.000375 \times 0.016129$

**$I = 6.05 \times 10^{-6}$ kg·m²** (or `6.05e-06`)

### What about the motor?
In your simulation, `propInertia` is technically handling the inertia of the *entire rotating assembly* (the prop + the motor bell). 
A typical 2207 motor bell weighs about 15 grams with a radius of about 14mm. Modeled as a thin cylinder ($I = mr^2$):

$I_{bell} = 0.015 \times (0.014)^2 = 2.94 \times 10^{-6}$ kg·m²

So the **total real-world inertia** of a motor + 5" prop is roughly **`8.9e-06` kg·m²**. 

Your value of `6.9e-06` is right in the sweet spot for a slightly lighter/more aggressive racing setup.




---



# Metrics


### 1. Thrust-to-Weight Ratio (TWR)
TWR is the absolute ceiling of the quad's performance. It dictates how fast the quad can arrest a fall and how "floaty" it feels when punching the throttle.
*   **How to calculate:** `(Max(Motor1Thrust) * 4) / (TotalQuadWeightInKg * 9.81)`
*   **Realistic Ranges:**
    *   **4:1 to 6:1:** Cinematic or heavy long-range quad. Feels sluggish, requires high throttle to maneuver.
    *   **7:1 to 10:1:** Standard Freestyle setup. Good balance of control and power.
    *   **11:1 to 15:1:** High-performance Racing setup. Very punchy, requires fine throttle control.
    *   **> 15:1:** Unrealistic for most setups, or represents an ultra-light "toothpick" build. Will feel overly twitchy in a simulator.

### 2. Hover Throttle Percentage
As you mentioned, this is where the stick sits when altitude is perfectly maintained. It determines your throttle resolution for upward vs. downward movements.
*   **How to calculate:** Calculate the thrust needed per motor for hover: `HoverThrustPerMotor = (QuadWeightInKg * 9.81) / 4`. Loop through the second half of your test (the linear `ThrustSweep`) and find the exact `Throttle` value where `ThrustSweep >= HoverThrustPerMotor`.
*   **Realistic Ranges:**
    *   **< 15%:** Too powerful/unrealistic. The pilot has virtually no stick travel before the quad rockets upward, making altitude management extremely difficult.
    *   **18% – 25%:** The sweet spot for standard 5" freestyle and racing.
    *   **30% – 40%:** Heavy cinematic rig (e.g., carrying a large cinema camera).
    *   **> 40%:** Underpowered, feels heavy, high risk of prop-wash and crashing out of dives.

### 3. Spool-Up Time (T90 Response Time)
This measures motor acceleration. In your first test, at exactly `t = 0.5s`, the oscillation sine wave hits `0`, and the throttle instantly steps to `1.0` (100%). This is a perfect 0-to-100% step test.
*   **How to calculate:** Find the time it takes from `t = 0.5s` for `Motor1Rpm` (or `Motor1Thrust`) to reach **90% of its absolute maximum value**. 
*   **Realistic Ranges:**
    *   **30ms – 60ms:** Very crisp and responsive (typical modern 5" racing motor, like a 2207 or 2306 on 6S).
    *   **60ms – 90ms:** Slightly soft, typical for larger motors (e.g., 7-inch props) or heavier bells.
    *   **> 100ms:** Feels highly sluggish. The pilot will notice a severe delay between moving the stick and the quad actually rotating or punching out. Unrealistic for a 5-inch racer.

### 4. Active Braking (Spool-Down Time)
Drones rely on ESC active braking to decelerate the props. If props spool down too slowly, the quad feels "floaty" and won't track sharp PID setpoint changes, leading to overshoot.
*   **How to calculate:** Look at the 4Hz oscillation phase (`0.0s` to `0.5s`). Measure the time delay (phase shift) between the `Throttle` dropping to its lowest point and the `Motor1Rpm` hitting its lowest point.
*   **Realistic Ranges:** Braking should take slightly longer than spool-up (since aerodynamic drag helps, but you are fighting the inertia of the prop). Usually **40ms to 80ms**. If it takes >150ms to spool down, active braking is likely configured incorrectly in the physics model.

### 5. Hover Efficiency (Grams per Watt)
This metric tells you if the aerodynamic and electrical math is realistic.
*   **How to calculate:** At the hover throttle index (calculated in metric #2), find the thrust in grams: `(Motor1Thrust / 9.81) * 1000`. Find the electrical power: `Motor1Current * BatteryVoltage`. Divide Grams by Watts.
*   **Realistic Ranges:**
    *   **5 to 9 g/W:** Very typical for 5" props at hover.
    *   **> 12 g/W:** Unrealistically efficient (perpetual motion territory for 5").
    *   **< 4 g/W:** Horribly inefficient. Either the aerodynamic drag of the prop is too high, or the motor electrical resistance is wrong.

### 6. Peak Current Draw
Determines if the electrical simulation is sane. A quad that draws too much current will trigger battery sag (if simulated) almost instantly.
*   **How to calculate:** Look at `Motor1Current` during the full throttle segment (0.5s - 1.0s). Multiply by 4 for total quad current.
*   **Realistic Ranges:**
    *   **100A – 160A total (25-40A per motor):** Normal for a high-performance 5" setup.
    *   **< 60A total:** Too low for a racing quad, more akin to a 3" micro.
    *   **> 220A total:** Very extreme, will likely destroy typical LiPo batteries in seconds.

### 7. Thrust Linearity (Curve Shape)
Aerodynamic thrust is proportional to the square of the RPM. Since RPM is roughly proportional to voltage (and thus throttle PWM), raw thrust curves up exponentially.
*   **How to calculate:** Look at your `ThrustSweep` data. Compare the thrust at 50% throttle to the thrust at 100% throttle. 
*   **Realistic Ranges:** If `ThrustSweep[at 50% throttle] == 0.25 * MaxThrust`, your raw physics are perfectly accurate. (Note: Betaflight has a feature called `thrust_linear` which alters the PID outputs to correct for this, but your raw open-loop motor response *should* be quadratic, not linear!).

### Summary Output Example

```text
--- QUADCOPTER REALISM REPORT ---
Weight:            650g
Hover Throttle:    22.4%       [PASS: Good for Freestyle]
Thrust/Weight:     8.4 : 1     [PASS: Punchy]
Spool-up (0-90%):  48 ms       [PASS: Crisp Response]
Max Current:       135 A       [PASS: Realistic for 6S LiPo]
Hover Efficiency:  6.2 g/W     [PASS: Sane Aerodynamics]
Raw Thrust Curve:  Quadratic   [PASS: Accurate Physics]
---------------------------------
Overall Feel: Crisp, responsive freestyle quad.
```