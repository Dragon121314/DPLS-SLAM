Binary Descriptors for Line Segments
====================================

This module shows how to extract line segments from an image by 2 different methods: First segmenting lines with Line Segment Detector LSDDetector and then (or just) using the Binary Descriptor to get the lines and give them a descriptor -- BinaryDescriptor. Finally, we can then match line segments using the BinaryDescriptorMatcher class.




     double A = kl.endPointY - kl.startPointY;      // A = y2 - y1
      double B = kl.startPointX - kl.endPointX;      // B = x1 - x2
      double C = kl.endPointX * kl.startPointY - kl.startPointX * kl.endPointY; // C = x2*y1 - x1*y2
      double D = 1.0/sqrt(pow(A,2)+pow(B,2));

      kl.equation=cv::Point3f(A*D,B*D,C*D);
      kl.ps=cv::Point2f(kl.startPointX,kl.startPointY);
      kl.pe=cv::Point2f(kl.endPointX,kl.endPointY);