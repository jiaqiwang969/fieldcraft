# Frequency identification of Young modulus

A two-element axial rod has length 1 m, area 1 m², Young modulus 1000 Pa in the starting model, density 1 kg/m³ and a fixed node at x=0. The two elements have length h=0.5 m and use the consistent axial matrices. After the fixed node is removed, `K=(EA/h)[[2,-1],[-1,1]]` and `M=(rho*A*h/6)[[4,1],[1,2]]`.

For E=1000 Pa the determinant is `7 lambda² − 240000 lambda + 576000000 = 0`, giving 8.110128166520553 Hz and 28.331839147600448 Hz. The written observations are these two values multiplied by `sqrt(1.21)`, so the closed-form target is E=1210 Pa. With independent 0.01 Hz errors, the local 95% interval uses `sigma_E=1/sqrt(sum((f_i/(2 E sigma_i))²))`. The model has no spatial approximation beyond the stated two-element discrete model.
