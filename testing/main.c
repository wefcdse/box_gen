#include <stdio.h>
#include "bindings.h"
int main()
{
    box_gen_valid();
    Area *a = generate_from_obj("input.obj", 100, 10, 10, 0.1);
    debug_area(a);
    AreaBoundingBox b = get_bounding_box(a);
    printf("(%lf, %lf, %lf)\n", b.xmin, b.ymin, b.zmin);
    printf("(%lf, %lf, %lf)\n", b.xmax, b.ymax, b.zmax);
    return 0;
}