function c = loadpcb442()
% LOADPCB442 Loads the data file. 
% NAME : pcb442
% COMMENT : Drilling problem (Groetschel/Juenger/Reinelt)
% TYPE : TSP
% DIMENSION : 442
% Source: www.iwr.uni-heidelberg.de/groups/comopt/software/TSPLIB95/tsp
clear all;
temp_x = [1 2.00000e+02 4.00000e+02
2 2.00000e+02 5.00000e+02
3 2.00000e+02 6.00000e+02
4 2.00000e+02 7.00000e+02
5 2.00000e+02 8.00000e+02
6 2.00000e+02 9.00000e+02
7 2.00000e+02 1.00000e+03
8 2.00000e+02 1.10000e+03
9 2.00000e+02 1.20000e+03
10 2.00000e+02 1.30000e+03
11 2.00000e+02 1.40000e+03
12 2.00000e+02 1.50000e+03
13 2.00000e+02 1.60000e+03
14 2.00000e+02 1.70000e+03
15 2.00000e+02 1.80000e+03
16 2.00000e+02 1.90000e+03
17 2.00000e+02 2.00000e+03
18 2.00000e+02 2.10000e+03
19 2.00000e+02 2.20000e+03
20 2.00000e+02 2.30000e+03
21 2.00000e+02 2.40000e+03
22 2.00000e+02 2.50000e+03
23 2.00000e+02 2.60000e+03
24 2.00000e+02 2.70000e+03
25 2.00000e+02 2.80000e+03
26 2.00000e+02 2.90000e+03
27 2.00000e+02 3.00000e+03
28 2.00000e+02 3.10000e+03
29 2.00000e+02 3.20000e+03
30 2.00000e+02 3.30000e+03
31 2.00000e+02 3.40000e+03
32 2.00000e+02 3.50000e+03
33 2.00000e+02 3.60000e+03
34 3.00000e+02 4.00000e+02
35 3.00000e+02 5.00000e+02
36 3.00000e+02 6.00000e+02
37 3.00000e+02 7.00000e+02
38 3.00000e+02 8.00000e+02
39 3.00000e+02 9.00000e+02
40 3.00000e+02 1.00000e+03
41 3.00000e+02 1.10000e+03
42 3.00000e+02 1.20000e+03
43 3.00000e+02 1.30000e+03
44 3.00000e+02 1.40000e+03
45 3.00000e+02 1.50000e+03
46 3.00000e+02 1.60000e+03
47 3.00000e+02 1.70000e+03
48 3.00000e+02 1.80000e+03
49 3.00000e+02 1.90000e+03
50 3.00000e+02 2.00000e+03
51 3.00000e+02 2.10000e+03
52 3.00000e+02 2.20000e+03
53 3.00000e+02 2.30000e+03
54 3.00000e+02 2.40000e+03
55 3.00000e+02 2.50000e+03
56 3.00000e+02 2.60000e+03
57 3.00000e+02 2.70000e+03
58 3.00000e+02 2.80000e+03
59 3.00000e+02 2.90000e+03
60 3.00000e+02 3.00000e+03
61 3.00000e+02 3.10000e+03
62 3.00000e+02 3.20000e+03
63 3.00000e+02 3.30000e+03
64 3.00000e+02 3.40000e+03
65 3.00000e+02 3.50000e+03
66 4.00000e+02 4.00000e+02
67 4.00000e+02 5.00000e+02
68 4.00000e+02 6.00000e+02
69 4.00000e+02 7.00000e+02
70 4.00000e+02 8.00000e+02
71 4.00000e+02 9.00000e+02
72 4.00000e+02 1.00000e+03
73 4.00000e+02 1.10000e+03
74 4.00000e+02 1.20000e+03
75 4.00000e+02 1.30000e+03
76 4.00000e+02 1.40000e+03
77 4.00000e+02 1.50000e+03
78 4.00000e+02 1.60000e+03
79 4.00000e+02 1.70000e+03
80 4.00000e+02 1.80000e+03
81 4.00000e+02 1.90000e+03
82 4.00000e+02 2.00000e+03
83 4.00000e+02 2.10000e+03
84 4.00000e+02 2.20000e+03
85 4.00000e+02 2.30000e+03
86 4.00000e+02 2.40000e+03
87 4.00000e+02 2.50000e+03
88 4.00000e+02 2.60000e+03
89 4.00000e+02 2.70000e+03
90 4.00000e+02 2.80000e+03
91 4.00000e+02 2.90000e+03
92 4.00000e+02 3.00000e+03
93 4.00000e+02 3.10000e+03
94 4.00000e+02 3.20000e+03
95 4.00000e+02 3.30000e+03
96 4.00000e+02 3.40000e+03
97 4.00000e+02 3.50000e+03
98 4.00000e+02 3.60000e+03
99 5.00000e+02 1.50000e+03
100 5.00000e+02 1.82900e+03
101 5.00000e+02 3.10000e+03
102 6.00000e+02 4.00000e+02
103 7.00000e+02 3.00000e+02
104 7.00000e+02 6.00000e+02
105 7.00000e+02 1.50000e+03
106 7.00000e+02 1.60000e+03
107 7.00000e+02 1.80000e+03
108 7.00000e+02 2.10000e+03
109 7.00000e+02 2.40000e+03
110 7.00000e+02 2.70000e+03
111 7.00000e+02 3.00000e+03
112 7.00000e+02 3.30000e+03
113 7.00000e+02 3.60000e+03
114 8.00000e+02 3.00000e+02
115 8.00000e+02 6.00000e+02
116 8.00000e+02 1.03000e+03
117 8.00000e+02 1.50000e+03
118 8.00000e+02 1.80000e+03
119 8.00000e+02 2.10000e+03
120 8.00000e+02 2.40000e+03
121 8.00000e+02 2.60000e+03
122 8.00000e+02 2.70000e+03
123 8.00000e+02 3.00000e+03
124 8.00000e+02 3.30000e+03
125 8.00000e+02 3.60000e+03
126 9.00000e+02 3.00000e+02
127 9.00000e+02 6.00000e+02
128 9.00000e+02 1.50000e+03
129 9.00000e+02 1.80000e+03
130 9.00000e+02 2.10000e+03
131 9.00000e+02 2.40000e+03
132 9.00000e+02 2.70000e+03
133 9.00000e+02 3.00000e+03
134 9.00000e+02 3.30000e+03
135 9.00000e+02 3.60000e+03
136 1.00000e+03 3.00000e+02
137 1.00000e+03 6.00000e+02
138 1.00000e+03 1.10000e+03
139 1.00000e+03 1.50000e+03
140 1.00000e+03 1.62900e+03
141 1.00000e+03 1.80000e+03
142 1.00000e+03 2.10000e+03
143 1.00000e+03 2.40000e+03
144 1.00000e+03 2.60000e+03
145 1.00000e+03 2.70000e+03
146 1.00000e+03 3.00000e+03
147 1.00000e+03 3.30000e+03
148 1.00000e+03 3.60000e+03
149 1.10000e+03 3.00000e+02
150 1.10000e+03 6.00000e+02
151 1.10000e+03 7.00000e+02
152 1.10000e+03 9.00000e+02
153 1.10000e+03 1.50000e+03
154 1.10000e+03 1.80000e+03
155 1.10000e+03 2.10000e+03
156 1.10000e+03 2.40000e+03
157 1.10000e+03 2.70000e+03
158 1.10000e+03 3.00000e+03
159 1.10000e+03 3.30000e+03
160 1.10000e+03 3.60000e+03
161 1.20000e+03 3.00000e+02
162 1.20000e+03 6.00000e+02
163 1.20000e+03 1.50000e+03
164 1.20000e+03 1.70000e+03
165 1.20000e+03 1.80000e+03
166 1.20000e+03 2.10000e+03
167 1.20000e+03 2.40000e+03
168 1.20000e+03 2.70000e+03
169 1.20000e+03 3.00000e+03
170 1.20000e+03 3.30000e+03
171 1.20000e+03 3.60000e+03
172 1.30000e+03 3.00000e+02
173 1.30000e+03 6.00000e+02
174 1.30000e+03 7.00000e+02
175 1.30000e+03 1.13000e+03
176 1.30000e+03 1.50000e+03
177 1.30000e+03 1.80000e+03
178 1.30000e+03 2.10000e+03
179 1.30000e+03 2.20000e+03
180 1.30000e+03 2.40000e+03
181 1.30000e+03 2.70000e+03
182 1.30000e+03 3.00000e+03
183 1.30000e+03 3.30000e+03
184 1.30000e+03 3.60000e+03
185 1.40000e+03 3.00000e+02
186 1.40000e+03 6.00000e+02
187 1.40000e+03 9.30000e+02
188 1.40000e+03 1.50000e+03
189 1.40000e+03 1.80000e+03
190 1.40000e+03 2.00000e+03
191 1.40000e+03 2.10000e+03
192 1.40000e+03 2.40000e+03
193 1.40000e+03 2.50000e+03
194 1.40000e+03 2.70000e+03
195 1.40000e+03 2.82000e+03
196 1.40000e+03 2.90000e+03
197 1.40000e+03 3.00000e+03
198 1.40000e+03 3.30000e+03
199 1.40000e+03 3.60000e+03
200 1.50000e+03 1.50000e+03
201 1.50000e+03 1.80000e+03
202 1.50000e+03 1.90000e+03
203 1.50000e+03 2.10000e+03
204 1.50000e+03 2.40000e+03
205 1.50000e+03 2.70000e+03
206 1.50000e+03 2.80000e+03
207 1.50000e+03 2.86000e+03
208 1.50000e+03 3.00000e+03
209 1.50000e+03 3.30000e+03
210 1.50000e+03 3.60000e+03
211 1.60000e+03 1.10000e+03
212 1.60000e+03 1.30000e+03
213 1.60000e+03 1.50000e+03
214 1.60000e+03 1.80000e+03
215 1.60000e+03 2.10000e+03
216 1.60000e+03 2.40000e+03
217 1.60000e+03 2.70000e+03
218 1.60000e+03 3.00000e+03
219 1.60000e+03 3.30000e+03
220 1.60000e+03 3.60000e+03
221 1.70000e+03 1.20000e+03
222 1.70000e+03 1.50000e+03
223 1.70000e+03 1.80000e+03
224 1.70000e+03 2.10000e+03
225 1.70000e+03 2.40000e+03
226 1.70000e+03 3.60000e+03
227 1.80000e+03 3.00000e+02
228 1.80000e+03 6.00000e+02
229 1.80000e+03 1.23000e+03
230 1.80000e+03 1.50000e+03
231 1.80000e+03 1.80000e+03
232 1.80000e+03 2.10000e+03
233 1.80000e+03 2.40000e+03
234 1.90000e+03 3.00000e+02
235 1.90000e+03 6.00000e+02
236 1.90000e+03 3.00000e+03
237 1.90000e+03 3.52000e+03
238 2.00000e+03 3.00000e+02
239 2.00000e+03 3.70000e+02
240 2.00000e+03 6.00000e+02
241 2.00000e+03 8.00000e+02
242 2.00000e+03 9.00000e+02
243 2.00000e+03 1.00000e+03
244 2.00000e+03 1.10000e+03
245 2.00000e+03 1.20000e+03
246 2.00000e+03 1.30000e+03
247 2.00000e+03 1.40000e+03
248 2.00000e+03 1.50000e+03
249 2.00000e+03 1.60000e+03
250 2.00000e+03 1.70000e+03
251 2.00000e+03 1.80000e+03
252 2.00000e+03 1.90000e+03
253 2.00000e+03 2.00000e+03
254 2.00000e+03 2.10000e+03
255 2.00000e+03 2.20000e+03
256 2.00000e+03 2.30000e+03
257 2.00000e+03 2.40000e+03
258 2.00000e+03 2.50000e+03
259 2.00000e+03 2.60000e+03
260 2.00000e+03 2.70000e+03
261 2.00000e+03 2.80000e+03
262 2.00000e+03 2.90000e+03
263 2.00000e+03 3.00000e+03
264 2.00000e+03 3.10000e+03
265 2.00000e+03 3.50000e+03
266 2.10000e+03 3.00000e+02
267 2.10000e+03 6.00000e+02
268 2.10000e+03 3.20000e+03
269 2.20000e+03 3.00000e+02
270 2.20000e+03 4.69000e+02
271 2.20000e+03 6.00000e+02
272 2.20000e+03 3.20000e+03
273 2.30000e+03 3.00000e+02
274 2.30000e+03 6.00000e+02
275 2.30000e+03 3.40000e+03
276 2.40000e+03 3.00000e+02
277 2.40000e+03 6.00000e+02
278 2.40000e+03 2.10000e+03
279 2.50000e+03 3.00000e+02
280 2.50000e+03 8.00000e+02
281 2.60000e+03 4.00000e+02
282 2.60000e+03 5.00000e+02
283 2.60000e+03 8.00000e+02
284 2.60000e+03 9.00000e+02
285 2.60000e+03 1.00000e+03
286 2.60000e+03 1.10000e+03
287 2.60000e+03 1.20000e+03
288 2.60000e+03 1.30000e+03
289 2.60000e+03 1.40000e+03
290 2.60000e+03 1.50000e+03
291 2.60000e+03 1.60000e+03
292 2.60000e+03 1.70000e+03
293 2.60000e+03 1.80000e+03
294 2.60000e+03 1.90000e+03
295 2.60000e+03 2.00000e+03
296 2.60000e+03 2.10000e+03
297 2.60000e+03 2.20000e+03
298 2.60000e+03 2.30000e+03
299 2.60000e+03 2.40000e+03
300 2.60000e+03 2.50000e+03
301 2.60000e+03 2.60000e+03
302 2.60000e+03 2.70000e+03
303 2.60000e+03 2.80000e+03
304 2.60000e+03 2.90000e+03
305 2.60000e+03 3.00000e+03
306 2.60000e+03 3.10000e+03
307 2.60000e+03 3.40000e+03
308 2.70000e+03 7.00000e+02
309 2.70000e+03 8.00000e+02
310 2.70000e+03 9.00000e+02
311 2.70000e+03 1.00000e+03
312 2.70000e+03 1.10000e+03
313 2.70000e+03 1.20000e+03
314 2.70000e+03 1.30000e+03
315 2.70000e+03 1.40000e+03
316 2.70000e+03 1.50000e+03
317 2.70000e+03 1.60000e+03
318 2.70000e+03 1.70000e+03
319 2.70000e+03 1.80000e+03
320 2.70000e+03 1.90000e+03
321 2.70000e+03 2.00000e+03
322 2.70000e+03 2.10000e+03
323 2.70000e+03 2.20000e+03
324 2.70000e+03 2.30000e+03
325 2.70000e+03 2.50000e+03
326 2.70000e+03 2.60000e+03
327 2.70000e+03 2.70000e+03
328 2.70000e+03 2.80000e+03
329 2.70000e+03 2.90000e+03
330 2.70000e+03 3.00000e+03
331 2.70000e+03 3.10000e+03
332 2.70000e+03 3.20000e+03
333 2.70000e+03 3.30000e+03
334 2.70000e+03 3.40000e+03
335 2.70000e+03 3.50000e+03
336 2.70000e+03 3.60000e+03
337 2.70000e+03 3.70000e+03
338 2.70000e+03 3.80000e+03
339 2.80000e+03 9.00000e+02
340 2.80000e+03 1.13000e+03
341 2.90000e+03 4.00000e+02
342 2.90000e+03 5.00000e+02
343 2.90000e+03 1.40000e+03
344 2.90000e+03 2.40000e+03
345 2.90000e+03 3.00000e+03
346 3.00000e+03 7.00000e+02
347 3.00000e+03 8.00000e+02
348 3.00000e+03 9.00000e+02
349 3.00000e+03 1.00000e+03
350 3.00000e+03 1.10000e+03
351 3.00000e+03 1.20000e+03
352 3.00000e+03 1.30000e+03
353 3.00000e+03 1.50000e+03
354 3.00000e+03 1.60000e+03
355 3.00000e+03 1.70000e+03
356 3.00000e+03 1.80000e+03
357 3.00000e+03 1.90000e+03
358 3.00000e+03 2.00000e+03
359 3.00000e+03 2.10000e+03
360 3.00000e+03 2.20000e+03
361 3.00000e+03 2.30000e+03
362 3.00000e+03 2.50000e+03
363 3.00000e+03 2.60000e+03
364 3.00000e+03 2.70000e+03
365 3.00000e+03 2.80000e+03
366 3.00000e+03 2.90000e+03
367 3.00000e+03 3.00000e+03
368 3.00000e+03 3.10000e+03
369 3.00000e+03 3.20000e+03
370 3.00000e+03 3.30000e+03
371 3.00000e+03 3.40000e+03
372 3.00000e+03 3.50000e+03
373 3.00000e+03 3.60000e+03
374 3.00000e+03 3.70000e+03
375 3.00000e+03 3.80000e+03
376 1.50000e+02 3.50000e+03
377 1.50000e+02 3.55000e+03
378 4.69000e+02 2.55000e+03
379 4.69000e+02 3.35000e+03
380 4.69000e+02 3.45000e+03
381 5.40000e+02 2.33000e+03
382 5.40000e+02 2.43000e+03
383 6.20000e+02 3.65000e+03
384 6.20000e+02 3.70900e+03
385 7.50000e+02 2.55000e+03
386 8.50000e+02 5.20000e+02
387 8.50000e+02 7.00000e+02
388 8.50000e+02 2.28000e+03
389 9.39000e+02 7.40000e+02
390 9.50000e+02 2.22000e+03
391 9.10000e+02 2.60000e+03
392 1.05000e+03 1.05000e+03
393 1.15000e+03 1.35000e+03
394 1.17000e+03 2.28000e+03
395 1.22000e+03 2.21000e+03
396 1.35000e+03 7.50000e+02
397 1.35000e+03 1.70000e+03
398 1.35000e+03 2.14000e+03
399 1.45000e+03 7.70000e+02
400 1.55000e+03 3.00000e+02
401 1.55000e+03 5.00000e+02
402 1.55000e+03 1.85000e+03
403 1.65000e+03 1.05000e+03
404 1.69000e+03 2.68000e+03
405 1.71000e+03 3.10000e+02
406 1.71000e+03 5.10000e+02
407 1.75000e+03 7.50000e+02
408 1.79000e+03 2.58000e+03
409 1.72000e+03 2.61000e+03
410 1.79000e+03 3.33000e+03
411 1.72000e+03 3.40900e+03
412 1.82900e+03 2.70000e+03
413 1.82900e+03 2.80000e+03
414 1.82900e+03 3.45000e+03
415 2.06000e+03 1.65000e+03
416 2.05000e+03 3.15000e+03
417 2.17000e+03 1.90000e+03
418 2.11000e+03 2.00000e+03
419 2.12000e+03 2.75000e+03
420 2.15000e+03 3.25000e+03
421 2.29000e+03 1.40000e+03
422 2.22000e+03 2.82000e+03
423 2.28000e+03 3.25000e+03
424 2.39000e+03 1.30000e+03
425 2.32000e+03 1.50000e+03
426 2.45000e+03 7.10000e+02
427 2.62000e+03 3.65000e+03
428 2.75000e+03 5.20000e+02
429 2.76000e+03 2.36000e+03
430 2.85000e+03 2.20000e+03
431 2.85000e+03 2.70000e+03
432 2.85000e+03 3.35000e+03
433 2.93000e+03 9.50000e+02
434 2.95000e+03 1.75000e+03
435 2.95000e+03 2.05000e+03
436 5.20000e+02 3.20000e+03
437 2.30000e+03 3.50000e+03
438 2.32000e+03 3.15000e+03
439 5.30000e+02 2.10000e+03
440 2.55000e+03 7.10000e+02
441 7.50000e+02 4.90000e+02
442 0.00000e+00 0.00000e+00];

cities = [temp_x(:,2)';temp_x(:,3)'];
save cities.mat cities -V6;