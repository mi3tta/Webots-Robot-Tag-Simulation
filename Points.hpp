#pragma once

class Points {
    public:
    static constexpr int VECTOR_ZERO = 10;

    Points();
    Points(double x, double y, double head);
    Points(double x, double y);


    double getX() const;
    double getY() const;
    double getH() const;

    void setX(double x);
    void setY(double y);
    void setH(double head);

    double directionBn(const Points &second) const;

    double angleAboveX() const;

    private:

    double mX {};
    double mY {};
    double mHead {};
};