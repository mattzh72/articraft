from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, sin, tau

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MM = 0.001


BOTTOM_T = 12.0
FOOT_T = 10.0
PLATE_T = 12.0
BOSS_DEPTH = 10.0
SIDE_WALL_T = 8.0
SIDE_WALL_H = 36.0
SHAFT_Z = 58.0
SHAFT_R = 6.0
BEARING_HOLE_R = 6.8
COLLAR_R = 9.5
COLLAR_W = 4.0
BOSS_R = 16.0
HOUSING_DEPTH = 120.0
PLATE_H = 96.0
SHAFT_SPAN = 108.0
GEAR_Y_CENTER = 0.0
BASE_DEPTH = 112.0


GEAR_SPECS = (
    {
        "part": "input_shaft",
        "joint": "housing_to_input",
        "tip_r": 20.0,
        "root_r": 16.5,
        "gear_w": 16.0,
        "hub_r": 11.0,
        "scallops": 12,
        "web_hole_r": 4.2,
        "front_stub": 16.0,
        "rear_stub": 0.0,
    },
    {
        "part": "countershaft",
        "joint": "housing_to_countershaft",
        "tip_r": 32.0,
        "root_r": 27.0,
        "gear_w": 18.0,
        "hub_r": 13.0,
        "scallops": 14,
        "web_hole_r": 5.5,
        "front_stub": 0.0,
        "rear_stub": 0.0,
    },
    {
        "part": "output_shaft",
        "joint": "housing_to_output",
        "tip_r": 44.0,
        "root_r": 38.5,
        "gear_w": 20.0,
        "hub_r": 15.0,
        "scallops": 18,
        "web_hole_r": 7.0,
        "front_stub": 0.0,
        "rear_stub": 18.0,
    },
)


def _shaft_x_positions() -> tuple[float, float, float]:
    first_gap = GEAR_SPECS[0]["tip_r"] + GEAR_SPECS[1]["tip_r"] + 2.0
    second_gap = GEAR_SPECS[1]["tip_r"] + GEAR_SPECS[2]["tip_r"] + 2.0
    left = -0.5 * (first_gap + second_gap)
    middle = left + first_gap
    right = middle + second_gap
    return (left, middle, right)


SHAFT_XS = _shaft_x_positions()
HOUSING_WIDTH = 2.0 * max(
    abs(x) + spec["tip_r"] + 18.0 for x, spec in zip(SHAFT_XS, GEAR_SPECS)
)


def _extrude_xz_profile(
    profile: cq.Workplane,
    y0: float,
    length: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
):
    return profile.extrude(length).translate((x, y0 + length, z))


def _cylinder_y(radius: float, y0: float, length: float, *, x: float = 0.0, z: float = 0.0):
    return _extrude_xz_profile(cq.Workplane("XZ").circle(radius), y0, length, x=x, z=z)


def _box_xyz(size_x: float, size_y: float, size_z: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((x, y, z0))
    )


def _make_gear_body(
    *,
    tip_r: float,
    root_r: float,
    width: float,
    hub_r: float,
    scallops: int,
    web_hole_r: float,
    y_center: float,
):
    y0 = y_center - 0.5 * width
    gear = _cylinder_y(tip_r, y0, width)

    scallop_r = max((tip_r - root_r) * 0.78, 2.6)
    scallop_center_r = root_r + 0.55 * (tip_r - root_r)
    scallop_points = [
        (
            scallop_center_r * cos(i * tau / scallops),
            scallop_center_r * sin(i * tau / scallops),
        )
        for i in range(scallops)
    ]
    scallop_cut = _extrude_xz_profile(
        cq.Workplane("XZ").pushPoints(scallop_points).circle(scallop_r),
        y0,
        width,
    )
    gear = gear.cut(scallop_cut)

    hub = _cylinder_y(hub_r, y0 - 3.0, width + 6.0)
    gear = gear.union(hub)

    web_points = [
        (
            0.52 * root_r * cos(i * tau / 3.0 + tau / 12.0),
            0.52 * root_r * sin(i * tau / 3.0 + tau / 12.0),
        )
        for i in range(3)
    ]
    web_cut = _extrude_xz_profile(
        cq.Workplane("XZ").pushPoints(web_points).circle(web_hole_r),
        y0 - 3.1,
        width + 6.2,
    )
    gear = gear.cut(web_cut)
    return gear


def _make_shaft_shape(spec: dict[str, float]) -> cq.Workplane:
    y0 = -0.5 * SHAFT_SPAN
    shaft = _cylinder_y(SHAFT_R, y0, SHAFT_SPAN)
    front_collar = _cylinder_y(COLLAR_R, y0 + 3.0, COLLAR_W)
    rear_collar = _cylinder_y(COLLAR_R, -y0 - COLLAR_W - 3.0, COLLAR_W)

    shape = shaft.union(front_collar).union(rear_collar)

    gear = _make_gear_body(
        tip_r=spec["tip_r"],
        root_r=spec["root_r"],
        width=spec["gear_w"],
        hub_r=spec["hub_r"],
        scallops=int(spec["scallops"]),
        web_hole_r=spec["web_hole_r"],
        y_center=GEAR_Y_CENTER,
    )
    return shape.union(gear)


def _make_bearing_pedestal(*, x: float, y_center: float, boss_length: float = 20.0) -> cq.Workplane:
    post = _box_xyz(22.0, boss_length - 2.0, SHAFT_Z - BOTTOM_T + 2.0, x=x, y=y_center, z0=BOTTOM_T)
    cap = _box_xyz(28.0, boss_length - 2.0, 8.0, x=x, y=y_center, z0=SHAFT_Z - 8.0)
    boss_y0 = y_center - 0.5 * boss_length
    boss = _cylinder_y(BOSS_R, boss_y0, boss_length, x=x, z=SHAFT_Z)
    bore = _cylinder_y(SHAFT_R + 0.2, boss_y0 - 0.5, boss_length + 1.0, x=x, z=SHAFT_Z)
    return post.union(cap).union(boss).cut(bore)


def _make_end_support(*, x: float, plate_y0: float, post_y: float) -> cq.Workplane:
    post = _box_xyz(20.0, 14.0, 40.0, x=x, y=post_y, z0=BOTTOM_T)
    plate_outer = _cylinder_y(14.0, plate_y0, 4.0, x=x, z=SHAFT_Z)
    plate_inner = _cylinder_y(SHAFT_R + 0.4, plate_y0 - 0.2, 4.4, x=x, z=SHAFT_Z)
    return post.union(plate_outer).cut(plate_inner)


def _make_housing_shape() -> cq.Workplane:
    body = _box_xyz(HOUSING_WIDTH, HOUSING_DEPTH, BOTTOM_T, z0=0.0)

    front_plate = _box_xyz(
        HOUSING_WIDTH,
        PLATE_T,
        PLATE_H,
        y=-0.5 * HOUSING_DEPTH + 0.5 * PLATE_T,
        z0=BOTTOM_T - 1.0,
    )
    rear_plate = _box_xyz(
        HOUSING_WIDTH,
        PLATE_T,
        PLATE_H,
        y=0.5 * HOUSING_DEPTH - 0.5 * PLATE_T,
        z0=BOTTOM_T - 1.0,
    )
    left_rail = _box_xyz(
        SIDE_WALL_T,
        HOUSING_DEPTH - 2.0 * PLATE_T,
        SIDE_WALL_H,
        x=-0.5 * HOUSING_WIDTH + 0.5 * SIDE_WALL_T,
        z0=BOTTOM_T - 1.0,
    )
    right_rail = _box_xyz(
        SIDE_WALL_T,
        HOUSING_DEPTH - 2.0 * PLATE_T,
        SIDE_WALL_H,
        x=0.5 * HOUSING_WIDTH - 0.5 * SIDE_WALL_T,
        z0=BOTTOM_T - 1.0,
    )
    body = body.union(front_plate).union(rear_plate).union(left_rail).union(right_rail)

    for x in SHAFT_XS:
        front_boss = _cylinder_y(BOSS_R, -0.5 * HOUSING_DEPTH - BOSS_DEPTH, BOSS_DEPTH + PLATE_T, x=x, z=SHAFT_Z)
        rear_boss = _cylinder_y(BOSS_R, 0.5 * HOUSING_DEPTH - PLATE_T, BOSS_DEPTH + PLATE_T, x=x, z=SHAFT_Z)
        body = body.union(front_boss).union(rear_boss)

    foot_x = 0.28 * HOUSING_WIDTH
    foot_y = 0.18 * HOUSING_DEPTH
    for x in (-foot_x, foot_x):
        foot = _box_xyz(52.0, 34.0, FOOT_T + 1.0, x=x, z0=-FOOT_T)
        body = body.union(foot)

    hole_points = []
    for x in SHAFT_XS:
        front_hole = _cylinder_y(
            BEARING_HOLE_R,
            -0.5 * HOUSING_DEPTH - BOSS_DEPTH - 0.2,
            BOSS_DEPTH + PLATE_T + 0.4,
            x=x,
            z=SHAFT_Z,
        )
        rear_hole = _cylinder_y(
            BEARING_HOLE_R,
            0.5 * HOUSING_DEPTH - PLATE_T - 0.2,
            BOSS_DEPTH + PLATE_T + 0.4,
            x=x,
            z=SHAFT_Z,
        )
        body = body.cut(front_hole).cut(rear_hole)

    for x in (-foot_x, foot_x):
        for y in (-foot_y, foot_y):
            hole_points.append((x, y))

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(4.2)
        .extrude(BOTTOM_T + FOOT_T + 0.4)
        .translate((0.0, 0.0, -FOOT_T - 0.2))
    )
    return body.cut(mount_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_reduction_gearbox")

    housing_color = model.material("housing_cast", rgba=(0.58, 0.60, 0.63, 1.0))
    steel_color = model.material("shaft_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH * MM, BASE_DEPTH * MM, BOTTOM_T * MM)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BOTTOM_T * MM)),
        material=housing_color,
        name="housing_base",
    )
    housing.visual(
        Box((44.0 * MM, 34.0 * MM, FOOT_T * MM)),
        origin=Origin(xyz=(-0.29 * HOUSING_WIDTH * MM, 0.0, -0.5 * FOOT_T * MM)),
        material=housing_color,
        name="left_foot",
    )
    housing.visual(
        Box((44.0 * MM, 34.0 * MM, FOOT_T * MM)),
        origin=Origin(xyz=(0.29 * HOUSING_WIDTH * MM, 0.0, -0.5 * FOOT_T * MM)),
        material=housing_color,
        name="right_foot",
    )
    housing.visual(
        Box((10.0 * MM, 84.0 * MM, 18.0 * MM)),
        origin=Origin(
            xyz=(
                (-0.5 * HOUSING_WIDTH + 18.0) * MM,
                0.0,
                (BOTTOM_T + 9.0) * MM,
            )
        ),
        material=housing_color,
        name="left_rail",
    )
    housing.visual(
        Box((10.0 * MM, 84.0 * MM, 18.0 * MM)),
        origin=Origin(
            xyz=(
                (0.5 * HOUSING_WIDTH - 18.0) * MM,
                0.0,
                (BOTTOM_T + 9.0) * MM,
            )
        ),
        material=housing_color,
        name="right_rail",
    )

    for side_name, plate_center_y in (
        ("front", -57.0),
        ("rear", 57.0),
    ):
        for shaft_name, x in zip(("input", "counter", "output"), SHAFT_XS):
            housing.visual(
                Box((20.0 * MM, 12.0 * MM, 36.0 * MM)),
                origin=Origin(
                    xyz=(
                        x * MM,
                        plate_center_y * MM,
                        (BOTTOM_T + 18.0) * MM,
                    )
                ),
                material=housing_color,
                name=f"{side_name}_{shaft_name}_support",
            )
            housing.visual(
                Box((28.0 * MM, 6.0 * MM, 28.0 * MM)),
                origin=Origin(
                    xyz=(
                        x * MM,
                        plate_center_y * MM,
                        SHAFT_Z * MM,
                    )
                ),
                material=housing_color,
                name=f"{side_name}_{shaft_name}_plate",
            )

    shaft_parts = []
    for index, (x, spec) in enumerate(zip(SHAFT_XS, GEAR_SPECS)):
        shaft = model.part(spec["part"])
        shaft_shape = _make_shaft_shape(spec)
        shaft.visual(
            mesh_from_cadquery(shaft_shape, f"{spec['part']}_assembly", unit_scale=MM),
            material=steel_color,
            name=f"{spec['part']}_assembly",
        )
        model.articulation(
            spec["joint"],
            ArticulationType.REVOLUTE,
            parent=housing,
            child=shaft,
            origin=Origin(xyz=(x * MM, 0.0, SHAFT_Z * MM)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=10.0,
                lower=-6.283,
                upper=6.283,
            ),
        )
        shaft_parts.append(shaft)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    countershaft = object_model.get_part("countershaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("housing_to_input")
    counter_joint = object_model.get_articulation("housing_to_countershaft")
    output_joint = object_model.get_articulation("housing_to_output")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 4,
        f"expected housing plus three shafts, got {len(object_model.parts)} parts",
    )
    ctx.check(
        "housing_is_only_root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "housing",
        "gearbox should root from one grounded housing part",
    )

    for joint in (input_joint, counter_joint, output_joint):
        ctx.check(
            f"{joint.name}_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be a revolute shaft joint",
        )
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should run along +Y, got {joint.axis}",
        )

    for shaft in (input_shaft, countershaft, output_shaft):
        ctx.expect_contact(shaft, housing, name=f"{shaft.name}_supported_by_housing")

    ctx.expect_origin_gap(
        countershaft,
        input_shaft,
        axis="x",
        min_gap=0.050,
        max_gap=0.058,
        name="input_to_countershaft_spacing",
    )
    ctx.expect_origin_gap(
        output_shaft,
        countershaft,
        axis="x",
        min_gap=0.074,
        max_gap=0.082,
        name="countershaft_to_output_spacing",
    )
    ctx.expect_origin_distance(
        input_shaft,
        countershaft,
        axes="z",
        max_dist=0.001,
        name="input_and_countershaft_parallel_height",
    )
    ctx.expect_origin_distance(
        countershaft,
        output_shaft,
        axes="z",
        max_dist=0.001,
        name="countershaft_and_output_parallel_height",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
