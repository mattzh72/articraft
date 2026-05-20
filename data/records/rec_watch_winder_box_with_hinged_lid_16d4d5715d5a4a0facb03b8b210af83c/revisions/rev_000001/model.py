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
    model = ArticulatedObject(name="watch_winder_presentation_box")

    outer_case = model.material("outer_case", rgba=(0.12, 0.08, 0.05, 1.0))
    inner_lining = model.material("inner_lining", rgba=(0.76, 0.69, 0.56, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.56, 0.62, 0.68, 0.28))
    satin_metal = model.material("satin_metal", rgba=(0.67, 0.68, 0.70, 1.0))
    dark_mechanism = model.material("dark_mechanism", rgba=(0.17, 0.18, 0.19, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.73, 0.66, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.240, 0.180, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=outer_case,
        name="base_plinth",
    )
    body.visual(
        Box((0.238, 0.178, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=inner_lining,
        name="floor_liner",
    )
    body.visual(
        Box((0.240, 0.014, 0.082)),
        origin=Origin(xyz=(0.0, 0.083, 0.057)),
        material=outer_case,
        name="front_wall",
    )
    body.visual(
        Box((0.240, 0.016, 0.094)),
        origin=Origin(xyz=(0.0, -0.082, 0.055)),
        material=outer_case,
        name="back_wall",
    )
    body.visual(
        Box((0.014, 0.150, 0.082)),
        origin=Origin(xyz=(-0.113, 0.000, 0.057)),
        material=outer_case,
        name="left_wall",
    )
    body.visual(
        Box((0.014, 0.150, 0.082)),
        origin=Origin(xyz=(0.113, 0.000, 0.057)),
        material=outer_case,
        name="right_wall",
    )
    body.visual(
        Box((0.198, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.067, 0.034)),
        material=inner_lining,
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.012, 0.080, 0.056)),
        origin=Origin(xyz=(-0.058, -0.019, 0.046)),
        material=dark_mechanism,
        name="left_cheek",
    )
    body.visual(
        Box((0.012, 0.080, 0.056)),
        origin=Origin(xyz=(0.058, -0.019, 0.046)),
        material=dark_mechanism,
        name="right_cheek",
    )
    body.visual(
        Box((0.116, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.043, 0.071)),
        material=dark_mechanism,
        name="cheek_bridge",
    )
    body.visual(
        Box((0.024, 0.020, 0.026)),
        origin=Origin(xyz=(-0.100, -0.085, 0.093)),
        material=outer_case,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.024, 0.020, 0.026)),
        origin=Origin(xyz=(0.100, -0.085, 0.093)),
        material=outer_case,
        name="right_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(-0.101, -0.087, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.101, -0.087, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.126)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.180, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, 0.008, 0.013)),
        material=outer_case,
        name="rear_rail",
    )
    lid.visual(
        Box((0.240, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, 0.169, 0.017)),
        material=outer_case,
        name="front_rail",
    )
    lid.visual(
        Box((0.014, 0.164, 0.034)),
        origin=Origin(xyz=(-0.113, 0.094, 0.017)),
        material=outer_case,
        name="left_lid_rail",
    )
    lid.visual(
        Box((0.014, 0.164, 0.034)),
        origin=Origin(xyz=(0.113, 0.094, 0.017)),
        material=outer_case,
        name="right_lid_rail",
    )
    lid.visual(
        Box((0.212, 0.160, 0.004)),
        origin=Origin(xyz=(0.0, 0.088, 0.028)),
        material=smoked_glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.212, 0.160, 0.004)),
        origin=Origin(xyz=(0.0, 0.088, 0.004)),
        material=inner_lining,
        name="inner_trim",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(-0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="left_hinge_sleeve",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="right_hinge_sleeve",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.034)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.090, 0.017)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.005, length=0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="spindle_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.034, length=0.086),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion_fabric,
        name="main_cushion",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(-0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mechanism,
        name="left_hub_cap",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mechanism,
        name="right_hub_cap",
    )
    cradle.visual(
        Box((0.080, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.000, 0.028)),
        material=cushion_fabric,
        name="retainer_pad",
    )
    cradle.visual(
        Box((0.072, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.006)),
        material=dark_mechanism,
        name="watch_stop",
    )
    cradle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.108),
        mass=0.28,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.087, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.42,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.004, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        name="closed lid covers the presentation box opening",
    )

    ctx.expect_gap(
        cradle,
        body,
        axis="x",
        positive_elem="main_cushion",
        negative_elem="left_cheek",
        min_gap=0.008,
        max_gap=0.011,
        name="left cheek frames the rotating cradle with a small clearance",
    )
    ctx.expect_gap(
        body,
        cradle,
        axis="x",
        positive_elem="right_cheek",
        negative_elem="main_cushion",
        min_gap=0.008,
        max_gap=0.011,
        name="right cheek frames the rotating cradle with a small clearance",
    )

    def aabb_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def aabb_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    closed_front_y = aabb_center_y(lid, "front_rail")
    closed_front_z = aabb_center_z(lid, "front_rail")
    with ctx.pose({lid_joint: 1.2}):
        open_front_y = aabb_center_y(lid, "front_rail")
        open_front_z = aabb_center_z(lid, "front_rail")
    ctx.check(
        "lid front rail swings upward when opened",
        closed_front_y is not None
        and closed_front_z is not None
        and open_front_y is not None
        and open_front_z is not None
        and open_front_z > closed_front_z + 0.10
        and open_front_y < closed_front_y - 0.08,
        details=(
            f"closed_front=(y={closed_front_y}, z={closed_front_z}), "
            f"open_front=(y={open_front_y}, z={open_front_z})"
        ),
    )

    cradle_origin_rest = ctx.part_world_position(cradle)
    stop_rest_y = aabb_center_y(cradle, "watch_stop")
    stop_rest_z = aabb_center_z(cradle, "watch_stop")
    with ctx.pose({cradle_joint: math.pi / 2.0}):
        cradle_origin_rotated = ctx.part_world_position(cradle)
        stop_rotated_y = aabb_center_y(cradle, "watch_stop")
        stop_rotated_z = aabb_center_z(cradle, "watch_stop")
    ctx.check(
        "cradle rotates about a fixed spindle axis",
        cradle_origin_rest is not None
        and cradle_origin_rotated is not None
        and stop_rest_y is not None
        and stop_rest_z is not None
        and stop_rotated_y is not None
        and stop_rotated_z is not None
        and abs(cradle_origin_rotated[0] - cradle_origin_rest[0]) < 1e-6
        and abs(cradle_origin_rotated[1] - cradle_origin_rest[1]) < 1e-6
        and abs(cradle_origin_rotated[2] - cradle_origin_rest[2]) < 1e-6
        and stop_rotated_z > stop_rest_z + 0.01
        and stop_rotated_y < stop_rest_y - 0.015,
        details=(
            f"origin_rest={cradle_origin_rest}, origin_rotated={cradle_origin_rotated}, "
            f"stop_rest=(y={stop_rest_y}, z={stop_rest_z}), "
            f"stop_rotated=(y={stop_rotated_y}, z={stop_rotated_z})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
