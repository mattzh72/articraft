from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_waste_bin")

    body_w = 0.43
    body_d = 0.34
    body_h = 0.82
    wall_t = 0.012
    base_t = 0.016

    lid_w = body_w + 0.006
    lid_d = body_d + 0.005
    lid_t = 0.018
    hinge_r = 0.010

    pedal_w = 0.31
    pedal_d = 0.095
    pedal_t = 0.014
    pedal_rest_tilt = 1.08
    pedal_rib_t = 0.003
    pedal_rib_d = 0.008

    def rotate_x(y: float, z: float, angle: float) -> tuple[float, float]:
        c = math.cos(angle)
        s = math.sin(angle)
        return (y * c - z * s, y * s + z * c)

    model.material("body_plastic", color=(0.88, 0.90, 0.90))
    model.material("lid_plastic", color=(0.67, 0.71, 0.74))
    model.material("pedal_rubber", color=(0.18, 0.19, 0.20))
    model.material("hinge_dark", color=(0.13, 0.13, 0.13))

    body = model.part("body")
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2 + wall_t / 2, 0.0, body_h / 2)),
        material="body_plastic",
        name="body_left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2 - wall_t / 2, 0.0, body_h / 2)),
        material="body_plastic",
        name="body_right_wall",
    )
    body.visual(
        Box((body_w - wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2 - wall_t / 2, body_h / 2)),
        material="body_plastic",
        name="body_front_wall",
    )
    body.visual(
        Box((body_w - wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2 + wall_t / 2, body_h / 2)),
        material="body_plastic",
        name="body_back_wall",
    )
    body.visual(
        Box((body_w - wall_t, body_d - wall_t, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2)),
        material="body_plastic",
        name="body_base_pan",
    )
    body.visual(
        Box((body_w - 0.08, 0.018, 0.012)),
        origin=Origin(
            xyz=(0.0, -body_d / 2 - 0.006, body_h - 0.012),
        ),
        material="hinge_dark",
        name="rear_hinge_beam",
    )
    body.visual(
        Box((body_w - 0.07, 0.03, 0.05)),
        origin=Origin(
            xyz=(0.0, body_d / 2 - 0.020, 0.11),
        ),
        material="body_plastic",
        name="front_pedal_guard",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d / 2 - 0.001, lid_t / 2)),
        material="lid_plastic",
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=lid_w - 0.05),
        origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
        material="hinge_dark",
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.16, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.258, 0.026)),
        material="hinge_dark",
        name="lid_pull",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.009, length=pedal_w - 0.03),
        origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
        material="hinge_dark",
        name="pedal_pivot_barrel",
    )
    plate_y, plate_z = rotate_x(pedal_d / 2, pedal_t / 2, pedal_rest_tilt)
    pedal.visual(
        Box((pedal_w, pedal_d, pedal_t)),
        origin=Origin(
            xyz=(0.0, plate_y, plate_z),
            rpy=(pedal_rest_tilt, 0.0, 0.0),
        ),
        material="pedal_rubber",
        name="pedal_plate",
    )
    for idx, rear_offset in enumerate((0.022, 0.040, 0.058, 0.076), start=1):
        rib_y, rib_z = rotate_x(
            rear_offset + pedal_rib_d / 2,
            pedal_t + pedal_rib_t / 2 - 0.0008,
            pedal_rest_tilt,
        )
        pedal.visual(
            Box((pedal_w - 0.05, pedal_rib_d, pedal_rib_t)),
            origin=Origin(
                xyz=(0.0, rib_y, rib_z),
                rpy=(pedal_rest_tilt, 0.0, 0.0),
            ),
            material="hinge_dark",
            name=f"pedal_tread_{idx}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_d / 2 - 0.004, body_h + 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=2.5,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, body_d / 2 + 0.012347, 0.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.5,
            lower=0.0,
            upper=0.48,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.28,
        name="closed lid covers the body opening footprint",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_front_wall",
        max_gap=0.020,
        max_penetration=0.0,
        name="closed lid sits just above the bin rim",
    )
    ctx.expect_origin_gap(
        pedal,
        body,
        axis="y",
        min_gap=0.16,
        max_gap=0.20,
        name="pedal pivot sits at the front of the body",
    )

    with ctx.pose({lid_hinge: 0.0}):
        lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.10}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid swings upward on the rear hinge",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.13
        and lid_open[1][1] < lid_closed[1][1] - 0.05,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    with ctx.pose({pedal_hinge: 0.0}):
        pedal_rest = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_hinge: 0.42}):
        pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal_plate")

    ctx.check(
        "pedal plate depresses downward",
        pedal_rest is not None
        and pedal_pressed is not None
        and pedal_pressed[0][2] < pedal_rest[0][2] - 0.015
        and pedal_pressed[0][1] > pedal_rest[0][1] - 0.005,
        details=f"rest={pedal_rest}, pressed={pedal_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
