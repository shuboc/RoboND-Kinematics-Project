/*
for (let i = 0; i < 7; ++i) {
    console.log(`
T${i}_${i+1} = Matrix([[cos(q${i+1}),                        -sin(q${i+1}),            0,              a${i}],
               [sin(q${i+1})*cos(alpha${i}), cos(q${i+1})*cos(alpha${i}), -sin(alpha${i}), -sin(alpha${i})*d${i+1}],
               [sin(q${i+1})*sin(alpha${i}), cos(q${i+1})*sin(alpha${i}),  cos(alpha${i}),  cos(alpha${i})*d${i+1}],
               [                  0,                   0,            0,               1]])
T${i}_${i+1} = T${i}_${i+1}.subs(s)`)
}
*/

/*
for (let i = 1; i < 7; ++i) {
    console.log(`T0_${i+1} = simplify(T0_${i} * T${i}_${i+1})`)
}
*/

for (let i = 1; i < 7; ++i) {
    console.log(`print("T0_${i} = ", T0_${i}.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))`)
}
