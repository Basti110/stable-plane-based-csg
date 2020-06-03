#include <nexus/test.hh>
#include <aabb.hh>

using namespace ob;

TEST("Test::Is_Ab_Greater_Cd") {
    i192 t1_1 = { 0xFF, 0xFF, 0xFF };
    i256 t1_2 = { 0xFF, 0xFF, 0xFF, 0xFF };
    i192 t1_3 = { 0xF, 0xF, 0xF };
    i256 t1_4 = { 0xFF, 0xFF, 0xFF, 0xFF };
    i192 t1_5 = { 0xF, 0xFF, 0xFF };
    i192 t1_6 = { 0xFF, 0xFF, 0xF };
    i256 t1_7 = { 0xFF, 0xFF, 0xF, 0xFF };
    i256 t1_8 = { 0xF, 0xFF, 0xFF, 0xFF };
    i256 t1_9 = { 0xFF, 0xFF, 0xFF, 0xFFF };
    i192 t1_10 = { 0x1, 0xFF, 0xFF };
    i192 t1_11 = { 0x2, 0xFF, 0xFF };
    i192 t1_12 = { 0x0, 0x0, 0x0 };

    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_3, t1_4) == 1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_5, t1_4) == 1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_6, t1_4) == 1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_1, t1_9) == -1);
    TG_ASSERT(ob::isAbGreaterCd(t1_3, t1_4, t1_1, t1_2) == -1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_3, t1_7) == 1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_2, t1_3, t1_8) == 1);
    TG_ASSERT(ob::isAbGreaterCd(t1_1, t1_8, t1_1, t1_2) == -1);
    TG_ASSERT(ob::isAbGreaterCd(t1_12, t1_2, t1_3, t1_8) == -1);
    TG_ASSERT(ob::isAbGreaterCd(t1_12, t1_2, t1_1, t1_4) == -1);
}

TEST("Test::Overlap_Interval") {
    Fraction<geometry128> t2_1 = { i256(1) , i192(3) };
    Fraction<geometry128> t2_2 = { i256(2) , i192(3) };
    Fraction<geometry128> t2_3 = { i256(6) , i192(9) };
    Fraction<geometry128> t2_4 = { i256(8) , i192(9) };
    Fraction<geometry128> t2_5 = { i256(7) , i192(9) };

    Fraction<geometry128> t2_1_1 = { { 0xFF, 0xFF, 0xFF, 1 } ,{ 0xFF, 0xFF, 3 } };
    Fraction<geometry128> t2_2_1 = { { 0xFF, 0xFF, 0xFF, 2 } ,{ 0xFF, 0xFF, 3 } };
    Fraction<geometry128> t2_3_1 = { { 0xFF, 0xFF, 0xFF, 2 } ,{ 0xFF, 0xFF, 3 } }; //TODO: t2_2_1 * 3 nehmen
    Fraction<geometry128> t2_4_1 = { { 0xFF, 0xFF, 0xFF, 8 } ,{ 0xFF, 0xFF, 9 } };
    Fraction<geometry128> t2_5_1 = { { 0xFF, 0xFF, 0xFF, 7 } ,{ 0xFF, 0xFF, 9 } };

    Fraction<geometry128> t2_1_2 = { { 1, 0xFF, 0xFF, 0xFF } ,{ 3, 0xFF, 0xFF } };
    Fraction<geometry128> t2_2_2 = { { 2, 0xFF, 0xFF, 0xFF } ,{ 3, 0xFF, 0xFF } };
    Fraction<geometry128> t2_3_2 = { { 2, 0xFF, 0xFF, 0xFF } ,{ 3, 0xFF, 0xFF } }; //TODO: t2_2_1 * 3 nehmen
    Fraction<geometry128> t2_4_2 = { { 8, 0xFF, 0xFF, 0xFF } ,{ 9, 0xFF, 0xFF } };
    Fraction<geometry128> t2_5_2 = { { 7, 0xFF, 0xFF, 0xFF } ,{ 9, 0xFF, 0xFF } };

    //Test all cases
    TG_ASSERT(overlapTest(t2_1, t2_2, t2_3, t2_4) == true);
    TG_ASSERT(overlapTest(t2_1, t2_2, t2_5, t2_4) == false);
    TG_ASSERT(overlapTest(t2_1, t2_5, t2_3, t2_4) == true);
    TG_ASSERT(overlapTest(t2_1, t2_4, t2_2, t2_5) == true);

    //switch intervals
    TG_ASSERT(overlapTest(t2_3, t2_4, t2_1, t2_2) == true);
    TG_ASSERT(overlapTest(t2_5, t2_4, t2_1, t2_2) == false);
    TG_ASSERT(overlapTest(t2_3, t2_4, t2_1, t2_5) == true);
    TG_ASSERT(overlapTest(t2_2, t2_5, t2_1, t2_4) == true);


    TG_ASSERT(overlapTest(t2_3_1, t2_4_1, t2_1_1, t2_2_1) == true);
    TG_ASSERT(overlapTest(t2_5_1, t2_4_1, t2_1_1, t2_2_1) == false);
    TG_ASSERT(overlapTest(t2_3_1, t2_4_1, t2_1_1, t2_5_1) == true);
    TG_ASSERT(overlapTest(t2_2_1, t2_5_1, t2_1_1, t2_4_1) == true);

    TG_ASSERT(overlapTest(t2_3_2, t2_4_2, t2_1_2, t2_2_2) == true);
    TG_ASSERT(overlapTest(t2_5_2, t2_4_2, t2_1_2, t2_2_2) == false);
    TG_ASSERT(overlapTest(t2_3_2, t2_4_2, t2_1_2, t2_5_2) == true);
    TG_ASSERT(overlapTest(t2_2_2, t2_5_2, t2_1_2, t2_4_2) == true);
}