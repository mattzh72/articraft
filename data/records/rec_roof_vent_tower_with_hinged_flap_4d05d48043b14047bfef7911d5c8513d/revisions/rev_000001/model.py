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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    darker_steel = model.material("weathered_steel", rgba=(0.52, 0.55, 0.58, 1.0))

    body_width = 0.240
    body_depth = 0.300
    body_height = 0.620
    wall_thickness = 0.020

    opening_width = 0.180
    opening_bottom = 0.312
    opening_top = 0.560
    opening_height = opening_top - opening_bottom

    housing = model.part("housing")
    housing.visual(
        Box((0.540, 0.440, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=darker_steel,
        name="roof_flashing",
    )
    housing.visual(
        Box((body_depth, wall_thickness, body_height - 0.012)),
        origin=Origin(
            xyz=(
                0.000,
                -(body_width * 0.5 - wall_thickness * 0.5),
                0.012 + (body_height - 0.012) * 0.5,
            )
        ),
        material=steel,
        name="left_wall",
    )
    housing.visual(
        Box((body_depth, wall_thickness, body_height - 0.012)),
        origin=Origin(
            xyz=(
                0.000,
                body_width * 0.5 - wall_thickness * 0.5,
                0.012 + (body_height - 0.012) * 0.5,
            )
        ),
        material=steel,
        name="right_wall",
    )
    housing.visual(
        Box((wall_thickness, body_width - 2.0 * wall_thickness, body_height - 0.012)),
        origin=Origin(
            xyz=(
                -(body_depth * 0.5 - wall_thickness * 0.5),
                0.000,
                0.012 + (body_height - 0.012) * 0.5,
            )
        ),
        material=steel,
        name="back_wall",
    )
    housing.visual(
        Box((wall_thickness, body_width - 2.0 * wall_thickness, opening_bottom - 0.012)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - wall_thickness * 0.5,
                0.000,
                0.012 + (opening_bottom - 0.012) * 0.5,
            )
        ),
        material=steel,
        name="front_lower_wall",
    )
    housing.visual(
        Box((wall_thickness, body_width - 2.0 * wall_thickness, 0.040)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 - wall_thickness * 0.5,
                0.000,
                opening_top + 0.020,
            )
        ),
        material=steel,
        name="opening_header_wall",
    )
    housing.visual(
        Box((body_depth + 0.040, body_width, 0.020)),
        origin=Origin(xyz=(0.020, 0.000, body_height - 0.010)),
        material=steel,
        name="roof_cap",
    )
    housing.visual(
        Box((0.016, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.008,
                -(opening_width * 0.5 + wall_thickness * 0.5),
                opening_bottom + opening_height * 0.5,
            )
        ),
        material=darker_steel,
        name="left_frame_lip",
    )
    housing.visual(
        Box((0.016, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.008,
                opening_width * 0.5 + wall_thickness * 0.5,
                opening_bottom + opening_height * 0.5,
            )
        ),
        material=darker_steel,
        name="right_frame_lip",
    )
    housing.visual(
        Box((0.016, opening_width + 2.0 * wall_thickness, 0.024)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.008,
                0.000,
                opening_top + 0.012,
            )
        ),
        material=darker_steel,
        name="opening_header_lip",
    )
    housing.visual(
        Box((0.022, 0.026, 0.016)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.008,
                -0.068,
                opening_top,
            )
        ),
        material=darker_steel,
        name="left_hinge_lug",
    )
    housing.visual(
        Box((0.022, 0.026, 0.016)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.008,
                0.068,
                opening_top,
            )
        ),
        material=darker_steel,
        name="right_hinge_lug",
    )
    housing.visual(
        Box((0.024, opening_width + 2.0 * wall_thickness, 0.024)),
        origin=Origin(
            xyz=(
                body_depth * 0.5 + 0.012,
                0.000,
                opening_bottom - 0.012,
            )
        ),
        material=darker_steel,
        name="outlet_stop_flange",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.540, 0.440, body_height)),
        mass=32.0,
        origin=Origin(xyz=(0.000, 0.000, body_height * 0.5)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.014, 0.188, 0.250)),
        origin=Origin(xyz=(0.009, 0.000, -0.125)),
        material=steel,
        name="flap_panel",
    )
    flap.visual(
        Box((0.024, 0.188, 0.024)),
        origin=Origin(xyz=(0.012, 0.000, -0.012)),
        material=darker_steel,
        name="flap_top_rail",
    )
    flap.visual(
        Cylinder(radius=0.007, length=0.196),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.030, 0.196, 0.250)),
        mass=4.0,
        origin=Origin(xyz=(0.010, 0.000, -0.120)),
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(body_depth * 0.5 + 0.026, 0.000, opening_top)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            positive_elem="flap_panel",
            negative_elem="outlet_stop_flange",
            min_gap=0.003,
            max_gap=0.008,
            name="closed flap sits just proud of the stop flange",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="y",
            elem_a="flap_panel",
            elem_b="opening_header_lip",
            min_overlap=0.180,
            name="flap spans the framed outlet width",
        )

    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    motion_ok = (
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > rest_aabb[1][0] + 0.080
        and open_aabb[1][2] > rest_aabb[1][2] + 0.010
    )
    ctx.check(
        "positive flap rotation swings outward and upward",
        motion_ok,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
