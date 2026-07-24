
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