L(1) = Link('d', 1.1, 'a', 0.6, 'alpha', -pi/2, 'qlim', [-150, 150] * pi/180);
L(2) = Link('d', 0, 'a', 1.465, 'alpha', 0, 'offset', -pi/2, 'qlim', [-40, 107.5] * pi/180);
L(3) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi, 'qlim', [-110, 145] * pi/180);
L(4) = Link('d', 1.6, 'a', 0, 'alpha', -pi/2, 'qlim', [-350, 350] * pi/180);
L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-118, 118] * pi/180);
L(6) = Link('d', 0.372, 'a', 0, 'alpha', 0, 'qlim', [-350, 350] * pi/180);

titan = SerialLink(L);
titan.teach()