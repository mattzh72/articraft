from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BevelGearPair,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _as_shape(obj):
    return obj.val() if hasattr(obj, "val") else obj


def _box(size, center):
    return cq.Workplane("XY", origin=center).box(*size)


def _x_cylinder(length, radius, center):
    return (
        cq.Workplane("YZ", origin=(center[0] - length / 2.0, center[1], center[2]))
        .circle(radius)
        .extrude(length)
    )


def _z_cylinder(length, radius, center):
    return (
        cq.Workplane("XY", origin=(center[0], center[1], center[2] - length / 2.0))
        .circle(radius)
        .extrude(length)
    )


def _union_all(*solids):
    result = cq.Workplane(obj=_as_shape(solids[0]))
    for solid in solids[1:]:
        result = result.union(_as_shape(solid))
    return result


def _spur_gear_along_x(teeth, tip_radius, root_radius, width, hub_radius, hub_length):
    points = []
    for i in range(teeth * 2):
        angle = pi * i / teeth
        radius = tip_radius if i % 2 == 0 else root_radius
        points.append((radius * cos(angle), radius * sin(angle)))

    gear_blank = (
        cq.Workplane("YZ", origin=(-width / 2.0, 0.0, 0.0))
        .moveTo(*points[0])
        .polyline(points[1:])
        .close()
        .extrude(width)
    )
    hub = _x_cylinder(hub_length, hub_radius, (0.0, 0.0, 0.0))
    return _as_shape(_union_all(gear_blank, hub))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_transfer_gearbox")

    housing_mat = model.material("housing_cast", rgba=(0.58, 0.60, 0.62, 1.0))
    shaft_mat = model.material("shaft_steel", rgba=(0.50, 0.52, 0.56, 1.0))
    gear_mat = model.material("gear_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    dark_mat = model.material("darkened_steel", rgba=(0.27, 0.28, 0.30, 1.0))

    housing_length = 0.46
    housing_depth = 0.24
    housing_height = 0.36
    wall = 0.022
    base = 0.024
    side_boss_len = 0.024
    top_boss_len = 0.032
    shaft_radius = 0.012
    thrust_washer_t = 0.004
    journal_radius = 0.010
    bearing_bore_radius = 0.013
    support_block_width = 0.040
    support_block_depth = 0.110
    support_block_height = 0.138

    input_axis_x = -0.115
    shaft_y = -0.015
    spur_center_x = 0.085

    layshaft_spur_teeth = 18
    output_spur_teeth = 26
    layshaft_spur_width = 0.024
    output_spur_width = 0.028
    layshaft_spur_tip_radius = 0.045
    layshaft_spur_root_radius = 0.039
    output_spur_tip_radius = 0.054
    output_spur_root_radius = 0.048
    layshaft_spur_pitch_radius = 0.042
    output_spur_pitch_radius = 0.052

    layshaft_spur_shape = _spur_gear_along_x(
        layshaft_spur_teeth,
        layshaft_spur_tip_radius,
        layshaft_spur_root_radius,
        layshaft_spur_width,
        hub_radius=0.016,
        hub_length=0.034,
    )
    output_spur_shape = _spur_gear_along_x(
        output_spur_teeth,
        output_spur_tip_radius,
        output_spur_root_radius,
        output_spur_width,
        hub_radius=0.017,
        hub_length=0.038,
    )
    output_spur_shape = _as_shape(
        cq.Workplane(obj=output_spur_shape).rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0 / output_spur_teeth
        )
    )
    spur_center_distance = layshaft_spur_pitch_radius + output_spur_pitch_radius

    layshaft_z = 0.201
    output_shaft_z = layshaft_z - spur_center_distance

    bevel_pair = BevelGearPair(
        module=0.0045,
        gear_teeth=18,
        pinion_teeth=18,
        face_width=0.018,
        axis_angle=90.0,
    )
    bevel_pair_shape = _as_shape(bevel_pair.build())
    bevel_solids = list(bevel_pair_shape.Solids())
    vertical_bevel = max(
        bevel_solids,
        key=lambda solid: solid.BoundingBox().xlen - solid.BoundingBox().zlen,
    )
    horizontal_bevel = max(
        bevel_solids,
        key=lambda solid: solid.BoundingBox().zlen - solid.BoundingBox().xlen,
    )
    bevel_apex_offset = float(bevel_pair.gear.cone_h)
    input_bevel_shape = _as_shape(
        cq.Workplane(obj=vertical_bevel).translate((0.0, 0.0, -bevel_apex_offset))
    )
    layshaft_bevel_shape = _as_shape(
        cq.Workplane(obj=horizontal_bevel).translate((0.0, 0.0, -bevel_apex_offset))
    )

    layshaft_left_support_x = -housing_length / 2.0 - side_boss_len
    layshaft_right_support_x = housing_length / 2.0 + side_boss_len
    output_left_support_x = layshaft_left_support_x
    output_right_support_x = layshaft_right_support_x
    input_top_support_z = housing_height + top_boss_len
    pedestal_center_x = housing_length / 2.0 - support_block_width / 2.0
    rear_wall_center_y = -housing_depth / 2.0 + wall / 2.0

    housing = _union_all(
        _box((housing_length, housing_depth, base), (0.0, 0.0, base / 2.0)),
        _box(
            (housing_length, wall, 0.190),
            (0.0, rear_wall_center_y, 0.095),
        ),
        _box(
            (support_block_width, support_block_depth, support_block_height),
            (-pedestal_center_x, shaft_y, support_block_height / 2.0),
        ),
        _box(
            (support_block_width, support_block_depth, support_block_height),
            (pedestal_center_x, shaft_y, support_block_height / 2.0),
        ),
        _box(
            (0.120, wall, 0.230),
            (input_axis_x, rear_wall_center_y, 0.115),
        ),
        _box(
            (0.210, 0.095, wall),
            (input_axis_x, -0.030, housing_height - wall / 2.0),
        ),
        _box(
            (0.120, 0.060, wall),
            (0.0, -0.030, 0.110 - wall / 2.0),
        ),
        _box(
            (0.080, 0.060, 0.080),
            (
                input_axis_x + 0.005,
                -0.030,
                0.070,
            ),
        ),
        _x_cylinder(
            side_boss_len,
            0.034,
            (-housing_length / 2.0 - side_boss_len / 2.0, shaft_y, layshaft_z),
        ),
        _x_cylinder(
            side_boss_len,
            0.034,
            (housing_length / 2.0 + side_boss_len / 2.0, shaft_y, layshaft_z),
        ),
        _x_cylinder(
            side_boss_len,
            0.032,
            (-housing_length / 2.0 - side_boss_len / 2.0, shaft_y, output_shaft_z),
        ),
        _x_cylinder(
            side_boss_len,
            0.032,
            (housing_length / 2.0 + side_boss_len / 2.0, shaft_y, output_shaft_z),
        ),
        _z_cylinder(
            top_boss_len,
            0.040,
            (input_axis_x, shaft_y, housing_height + top_boss_len / 2.0),
        ),
    )
    housing = housing.cut(
        _x_cylinder(
            housing_length + 2.0 * side_boss_len + 0.020,
            bearing_bore_radius,
            (0.0, shaft_y, layshaft_z),
        )
    )
    housing = housing.cut(
        _x_cylinder(
            housing_length + 2.0 * side_boss_len + 0.020,
            bearing_bore_radius,
            (0.0, shaft_y, output_shaft_z),
        )
    )
    housing = housing.cut(
        _z_cylinder(
            housing_height + top_boss_len + 0.020,
            bearing_bore_radius,
            (
                input_axis_x,
                shaft_y,
                (housing_height + top_boss_len) / 2.0,
            ),
        )
    )

    input_body_shape = _union_all(
        _z_cylinder(0.210, journal_radius, (0.0, 0.0, 0.087)),
        _z_cylinder(0.016, 0.0125, (0.0, 0.0, -0.021)),
        _z_cylinder(0.024, 0.014, (0.0, 0.0, -0.038)),
        _z_cylinder(
            thrust_washer_t,
            0.019,
            (0.0, 0.0, input_top_support_z - layshaft_z + thrust_washer_t / 2.0),
        ),
        _z_cylinder(0.046, 0.016, (0.0, 0.0, 0.214)),
    )

    layshaft_origin_x = input_axis_x
    layshaft_left_local = layshaft_left_support_x - layshaft_origin_x
    layshaft_right_local = layshaft_right_support_x - layshaft_origin_x
    layshaft_spur_local_x = spur_center_x - layshaft_origin_x
    layshaft_body_shape = _union_all(
        _x_cylinder(
            0.095,
            journal_radius,
            (-0.0915, 0.0, 0.0),
        ),
        _x_cylinder(
            0.394,
            journal_radius,
            (0.172, 0.0, 0.0),
        ),
        _x_cylinder(
            thrust_washer_t,
            0.019,
            (layshaft_left_local - thrust_washer_t / 2.0, 0.0, 0.0),
        ),
        _x_cylinder(
            thrust_washer_t,
            0.019,
            (layshaft_right_local + thrust_washer_t / 2.0, 0.0, 0.0),
        ),
        _x_cylinder(0.034, 0.016, (layshaft_spur_local_x + 0.034, 0.0, 0.0)),
    )
    layshaft_spur_shape = _as_shape(
        cq.Workplane(obj=layshaft_spur_shape).translate((layshaft_spur_local_x, 0.0, 0.0))
    )

    output_origin_x = spur_center_x
    output_left_local = output_left_support_x - output_origin_x
    output_right_local = output_right_support_x - output_origin_x
    output_body_shape = _union_all(
        _x_cylinder(0.325, journal_radius, (-0.1765, 0.0, 0.0)),
        _x_cylinder(0.161, journal_radius, (0.0945, 0.0, 0.0)),
        _x_cylinder(
            thrust_washer_t,
            0.020,
            (output_left_local - thrust_washer_t / 2.0, 0.0, 0.0),
        ),
        _x_cylinder(
            thrust_washer_t,
            0.020,
            (output_right_local + thrust_washer_t / 2.0, 0.0, 0.0),
        ),
        _x_cylinder(0.028, 0.016, (0.014, 0.0, 0.0)),
        _x_cylinder(0.088, 0.017, (0.219, 0.0, 0.0)),
        _x_cylinder(0.012, 0.032, (0.269, 0.0, 0.0)),
    )

    housing_part = model.part("housing")
    housing_part.visual(
        mesh_from_cadquery(_as_shape(housing), "gearbox_housing"),
        material=housing_mat,
        name="housing_shell",
    )

    input_part = model.part("input_shaft")
    input_part.visual(
        mesh_from_cadquery(_as_shape(input_body_shape), "input_shaft_body"),
        material=shaft_mat,
        name="input_shaft_body",
    )
    input_part.visual(
        mesh_from_cadquery(input_bevel_shape, "input_bevel_gear"),
        material=gear_mat,
        name="input_bevel_gear",
    )

    layshaft_part = model.part("layshaft")
    layshaft_part.visual(
        mesh_from_cadquery(_as_shape(layshaft_body_shape), "layshaft_body"),
        material=shaft_mat,
        name="layshaft_body",
    )
    layshaft_part.visual(
        mesh_from_cadquery(layshaft_bevel_shape, "layshaft_bevel_gear"),
        material=gear_mat,
        name="layshaft_bevel_gear",
    )
    layshaft_part.visual(
        mesh_from_cadquery(layshaft_spur_shape, "layshaft_spur_gear"),
        material=gear_mat,
        name="layshaft_spur_gear",
    )

    output_part = model.part("output_shaft")
    output_part.visual(
        mesh_from_cadquery(_as_shape(output_body_shape), "output_shaft_body"),
        material=dark_mat,
        name="output_shaft_body",
    )
    output_part.visual(
        mesh_from_cadquery(output_spur_shape, "output_spur_gear"),
        material=gear_mat,
        name="output_spur_gear",
    )

    model.articulation(
        "housing_to_input_shaft",
        ArticulationType.REVOLUTE,
        parent=housing_part,
        child=input_part,
        origin=Origin(xyz=(input_axis_x, shaft_y, layshaft_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "housing_to_layshaft",
        ArticulationType.REVOLUTE,
        parent=housing_part,
        child=layshaft_part,
        origin=Origin(xyz=(layshaft_origin_x, shaft_y, layshaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "housing_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=housing_part,
        child=output_part,
        origin=Origin(xyz=(output_origin_x, shaft_y, output_shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )

    return model
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    layshaft = object_model.get_part("layshaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("housing_to_input_shaft")
    layshaft_joint = object_model.get_articulation("housing_to_layshaft")
    output_joint = object_model.get_articulation("housing_to_output_shaft")

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
    ctx.allow_overlap(
        housing,
        input_shaft,
        elem_a="housing_shell",
        elem_b="input_shaft_body",
        reason="Open housing uses simplified bearing support geometry around the vertical shaft.",
    )
    ctx.allow_overlap(
        housing,
        layshaft,
        elem_a="housing_shell",
        elem_b="layshaft_body",
        reason="Layshaft journals are represented through simplified bearing carriers within the housing mesh.",
    )
    ctx.allow_overlap(
        housing,
        output_shaft,
        elem_a="housing_shell",
        elem_b="output_shaft_body",
        reason="Output shaft passes through simplified bearing carriers modeled as one housing mesh.",
    )
    ctx.allow_overlap(
        input_shaft,
        layshaft,
        elem_a="input_shaft_body",
        elem_b="layshaft_body",
        reason="The bevel stage is simplified around the virtual cone apex, so hidden shaft volumes are allowed to coincide.",
    )
    ctx.allow_overlap(
        input_shaft,
        layshaft,
        elem_a="input_bevel_gear",
        elem_b="layshaft_bevel_gear",
        reason="Bevel gear teeth intentionally intermesh at the right-angle stage.",
    )
    ctx.allow_overlap(
        layshaft,
        output_shaft,
        elem_a="layshaft_spur_gear",
        elem_b="output_spur_gear",
        reason="Spur gear teeth intentionally intermesh.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all requested shaft joints are revolute",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (input_joint, layshaft_joint, output_joint)
        ),
        details="Input shaft, layshaft, and output shaft should each be their own revolute joint.",
    )
    ctx.check(
        "joint axes match gearbox shafts",
        input_joint.axis == (0.0, 0.0, 1.0)
        and layshaft_joint.axis == (1.0, 0.0, 0.0)
        and output_joint.axis == (1.0, 0.0, 0.0),
        details="Expected one vertical input axis and two horizontal shaft axes.",
    )
    ctx.check(
        "bevel shaft axes intersect",
        input_joint.origin.xyz == layshaft_joint.origin.xyz,
        details="The vertical input shaft and horizontal layshaft should meet at the bevel mesh apex.",
    )
    ctx.check(
        "spur shaft spacing is realistic",
        abs(layshaft_joint.origin.xyz[1] - output_joint.origin.xyz[1]) < 1e-6
        and abs(layshaft_joint.origin.xyz[2] - output_joint.origin.xyz[2] - 0.094) < 0.003,
        details="The spur pair should sit on parallel horizontal shafts with about 94 mm center distance.",
    )

    ctx.expect_contact(input_shaft, housing, name="input shaft is supported by the housing")
    ctx.expect_contact(layshaft, housing, name="layshaft is supported by the housing")
    ctx.expect_contact(output_shaft, housing, name="output shaft is supported by the housing")
    ctx.expect_gap(
        layshaft,
        output_shaft,
        axis="z",
        positive_elem="layshaft_spur_gear",
        negative_elem="output_spur_gear",
        min_gap=-0.010,
        max_gap=0.003,
        name="spur gears are close enough to mesh",
    )
    ctx.expect_contact(
        input_shaft,
        layshaft,
        elem_a="input_bevel_gear",
        elem_b="layshaft_bevel_gear",
        name="bevel gears mesh at the right-angle stage",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
