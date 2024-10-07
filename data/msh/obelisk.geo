lcar1 = 1;

Point(newp) = {3.281868, 0.015643, 0.500000, lcar1}; /* Point      1 */
Point(newp) = {3.126076, 0.999275, 0.500000, lcar1}; /* Point      2 */
Point(newp) = {3.281868, 0.015643, -0.500000, lcar1}; /* Point      4 */
Point(newp) = {2.326938, 0.015643, -0.350000, lcar1}; /* Point      8 */
Point(newp) = {2.326938, 0.015643, 0.350000, lcar1}; /* Point      5 */
Point(newp) = {2.217884, 0.704186, 0.350000, lcar1}; /* Point      6 */
Point(newp) = {3.126076, 0.999275, -0.500000, lcar1}; /* Point      3 */
Point(newp) = {2.217884, 0.704186, -0.350000, lcar1}; /* Point      7 */

n = 0;
t = 0;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};

Point(newp) = {3.116408, 1.029031, 0.500000, lcar1};
Point(newp) = {2.664282, 1.916378, 0.500000, lcar1};
Point(newp) = {3.116408, 1.029031, -0.500000, lcar1};
Point(newp) = {2.208215, 0.733941, -0.350000, lcar1};
Point(newp) = {2.208215, 0.733941, 0.350000, lcar1};
Point(newp) = {1.891727, 1.355084, 0.350000, lcar1};
Point(newp) = {2.664282, 1.916378, -0.500000, lcar1};
Point(newp) = {1.891727, 1.355084, -0.350000, lcar1};
c = 1;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {2.645892, 1.941689, 0.500000, lcar1};
Point(newp) = {1.941689, 2.645892, 0.500000, lcar1};
Point(newp) = {2.645892, 1.941689, -0.500000, lcar1};
Point(newp) = {1.873337, 1.380396, -0.350000, lcar1};
Point(newp) = {1.873337, 1.380396, 0.350000, lcar1};
Point(newp) = {1.380396, 1.873337, 0.350000, lcar1};
Point(newp) = {1.941689, 2.645892, -0.500000, lcar1};
Point(newp) = {1.380396, 1.873337, -0.350000, lcar1};
c = 2;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};

Point(newp) = {1.916378, 2.664282, 0.500000, lcar1};
Point(newp) = {1.029031, 3.116408, 0.500000, lcar1};
Point(newp) = {1.916378, 2.664282, -0.500000, lcar1};
Point(newp) = {1.355084, 1.891727, -0.350000, lcar1};
Point(newp) = {1.355084, 1.891727, 0.350000, lcar1};
Point(newp) = {0.733941, 2.208215, 0.350000, lcar1};
Point(newp) = {1.029031, 3.116408, -0.500000, lcar1};
Point(newp) = {0.733941, 2.208215, -0.350000, lcar1};

c = 3;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {0.999275, 3.126076, 0.500000, lcar1};
Point(newp) = {0.015643, 3.281868, 0.500000, lcar1};
Point(newp) = {0.999275, 3.126076, -0.500000, lcar1};
Point(newp) = {0.704186, 2.217884, -0.350000, lcar1};
Point(newp) = {0.704186, 2.217884, 0.350000, lcar1};
Point(newp) = {0.015643, 2.326938, 0.350000, lcar1};
Point(newp) = {0.015643, 3.281868, -0.500000, lcar1};
Point(newp) = {0.015643, 2.326938, -0.350000, lcar1};

c = 4;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};

Point(newp) = {-0.015643, 3.281868, 0.500000, lcar1};
Point(newp) = {-0.999275, 3.126076, 0.500000, lcar1};
Point(newp) = {-0.015643, 3.281868, -0.500000, lcar1};
Point(newp) = {-0.015643, 2.326938, -0.350000, lcar1};
Point(newp) = {-0.015643, 2.326938, 0.350000, lcar1};
Point(newp) = {-0.704186, 2.217884, 0.350000, lcar1};
Point(newp) = {-0.999275, 3.126076, -0.500000, lcar1};
Point(newp) = {-0.704186, 2.217884, -0.350000, lcar1};

c = 5;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {-1.029031, 3.116408, 0.500000, lcar1};
Point(newp) = {-1.916378, 2.664282, 0.500000, lcar1};
Point(newp) = {-1.029031, 3.116408, -0.500000, lcar1};
Point(newp) = {-0.733941, 2.208215, -0.350000, lcar1};
Point(newp) = {-0.733941, 2.208215, 0.350000, lcar1};
Point(newp) = {-1.355084, 1.891727, 0.350000, lcar1};
Point(newp) = {-1.916378, 2.664282, -0.500000, lcar1};
Point(newp) = {-1.355084, 1.891727, -0.350000, lcar1};

c = 6;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {-1.941689, 2.645892, 0.500000, lcar1};
Point(newp) = {-2.645892, 1.941689, 0.500000, lcar1};
Point(newp) = {-1.941689, 2.645892, -0.500000, lcar1};
Point(newp) = {-1.380396, 1.873337, -0.350000, lcar1};
Point(newp) = {-1.380396, 1.873337, 0.350000, lcar1};
Point(newp) = {-1.873337, 1.380396, 0.350000, lcar1};
Point(newp) = {-2.645892, 1.941689, -0.500000, lcar1};
Point(newp) = {-1.873337, 1.380396, -0.350000, lcar1};

c = 7;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {-2.664282, 1.916378, 0.500000, lcar1};
Point(newp) = {-3.116408, 1.029031, 0.500000, lcar1};
Point(newp) = {-2.664282, 1.916378, -0.500000, lcar1};
Point(newp) = {-1.891727, 1.355084, -0.350000, lcar1};
Point(newp) = {-1.891727, 1.355084, 0.350000, lcar1};
Point(newp) = {-2.208215, 0.733941, 0.350000, lcar1};
Point(newp) = {-3.116408, 1.029031, -0.500000, lcar1};
Point(newp) = {-2.208215, 0.733941, -0.350000, lcar1};

c = 8;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};


Point(newp) = {-3.126076, 0.999275, 0.500000, lcar1};
Point(newp) = {-3.281868, 0.015643, 0.500000, lcar1};
Point(newp) = {-3.126076, 0.999275, -0.500000, lcar1};
Point(newp) = {-2.217884, 0.704186, -0.350000, lcar1};
Point(newp) = {-2.217884, 0.704186, 0.350000, lcar1};
Point(newp) = {-2.326938, 0.015643, 0.350000, lcar1};
Point(newp) = {-3.281868, 0.015643, -0.500000, lcar1};
Point(newp) = {-2.326938, 0.015643, -0.350000, lcar1};

c = 9;
n = 8 * c;
t = 26 * c;
Line(t + 1) = {n + 3,n + 1};
Line(t + 2) = {n + 3, n + 7};
Line(t + 3) = {n + 7, n + 2};
Line(t + 4) = {n + 2, n + 1};
Line(t + 5) = {n + 1, n + 5};
Line(t + 6) = {n + 5, n + 4};
Line(t + 7) = {n + 4, n + 8};
Line(t + 8) = {n + 8, n + 6};
Line(t + 9) = {n + 6, n + 5};
Line(t + 10) = {n + 6, n + 2};
Line(t + 11) = {n + 3, n + 4};
Line(t + 12) = {n + 8, n + 7};

Line Loop(t +13) = {- t - 6,- t - 5,-t - 1,t + 11};
Plane Surface(t +14) = {t +13};
Line Loop(t +15) = {t + 4, t +5,-t - 9,t + 10};
Plane Surface(t +16) = {t +15};
Line Loop(t +17) = {- t -3,- t -12,t + 8,t + 10};
Plane Surface(t +18) = {t +17};
Line Loop(t +19) = {t + 7, t + 12,-t - 2,t + 11};
Plane Surface(t +20) = {t +19};
Line Loop(t +21) = {-t - 4,-t -3,-t - 2,t + 1};
Plane Surface(t +22) = {t +21};
Line Loop(t +23) = {t + 8,t + 9,t + 6,t + 7};
Plane Surface(t +24) = {t +23};

Surface Loop(t + 25) = {t +14,t +24,-t - 18,t +22,t +16,-t - 20};
Volume(t + 26) = {t + 25};