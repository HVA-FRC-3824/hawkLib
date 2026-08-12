// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared;

public class Quadruple<A, B, C, D> {
  private final A first;
  private final B second;
  private final C third;
  private final D fourth;

  public Quadruple(A first, B second, C third, D fourth) {
    this.first = first;
    this.second = second;
    this.third = third;
    this.fourth = fourth;
  }

  public A getFirst() {
    return first;
  }

  public B getSecond() {
    return second;
  }

  public C getThird() {
    return third;
  }

  public D getFourth() {
    return fourth;
  }
}
