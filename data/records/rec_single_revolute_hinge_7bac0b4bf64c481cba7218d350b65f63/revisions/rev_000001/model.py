from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.0032
LEAF_W = 0.042
GROUND_LEN = 0.050
MOVING_LEN = 0.060
CHEEK_W = 0.011
KNUCKLE_W = 0.018
CHEEK_CENTER_Y = 0.0155
KNUCKLE_R = 0.0055
PIN_R = 0.0031
GROUND_HOLE_R = 0.0034
PIN_HEAD_R = 0.0043
PIN_HEAD_T = 0.0016


def _extrude_centered(profile: cq.Workplane, thickness: float) -> cq.Workplane:
    return profile.extrude(thickness).translate((0.0, 0.0, -(thickness / 2.0)))


def _cylinder_along_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _ground_leaf_shape() -> cq.Workplane:
    plate = _extrude_centered(
        cq.Workplane("XY").center(-0.029, 0.0).rect(0.042, LEAF_W),
        PLATE_T,
    )
    plate = plate.cut(
        cq.Workplane("XY")
        .pushPoints([(-0.038, 0.0), (-0.022, 0.0)])
        .circle(0.0027)
        .extrude(PLATE_T + 0.002)
        .translate((0.0, 0.0, -((PLATE_T + 0.002) / 2.0)))
    )

    upper_bridge = _extrude_centered(
        cq.Workplane("XY").center(-0.008, CHEEK_CENTER_Y).rect(0.016, CHEEK_W),
        PLATE_T,
    )
    lower_bridge = _extrude_centered(
        cq.Workplane("XY").center(-0.008, -CHEEK_CENTER_Y).rect(0.016, CHEEK_W),
        PLATE_T,
    )

    upper_cheek = _cylinder_along_y(KNUCKLE_R, CHEEK_W).cut(
        _cylinder_along_y(GROUND_HOLE_R, CHEEK_W + 0.002)
    ).translate((0.0, CHEEK_CENTER_Y, 0.0))
    lower_cheek = _cylinder_along_y(KNUCKLE_R, CHEEK_W).cut(
        _cylinder_along_y(GROUND_HOLE_R, CHEEK_W + 0.002)
    ).translate((0.0, -CHEEK_CENTER_Y, 0.0))

    return (
        plate.union(upper_bridge)
        .union(lower_bridge)
        .union(upper_cheek)
        .union(lower_cheek)
    )


def _moving_leaf_shape() -> cq.Workplane:
    profile = cq.Workplane("XY").polyline(
        [
            (0.0048, -(KNUCKLE_W / 2.0)),
            (0.016, -(KNUCKLE_W / 2.0)),
            (0.026, -(LEAF_W / 2.0)),
            (MOVING_LEN, -(LEAF_W / 2.0)),
            (MOVING_LEN, LEAF_W / 2.0),
            (0.026, LEAF_W / 2.0),
            (0.016, KNUCKLE_W / 2.0),
            (0.0048, KNUCKLE_W / 2.0),
        ]
    ).close()
    leaf = _extrude_centered(profile, PLATE_T)
    leaf_holes = (
        cq.Workplane("XY")
        .pushPoints([(0.032, 0.0), (0.048, 0.0)])
        .circle(0.0027)
        .extrude(PLATE_T + 0.002)
        .translate((0.0, 0.0, -((PLATE_T + 0.002) / 2.0)))
    )
    leaf = leaf.cut(leaf_holes)

    knuckle = _cylinder_along_y(KNUCKLE_R, KNUCKLE_W).cut(
        _cylinder_along_y(PIN_R, KNUCKLE_W + 0.006)
    )

    return leaf.union(knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clevis_panel_hinge")

    model.material("ground_leaf_finish", color=(0.33, 0.35, 0.38, 1.0))
    model.material("pin_finish", color=(0.54, 0.55, 0.57, 1.0))
    model.material("moving_leaf_finish", color=(0.73, 0.74, 0.76, 1.0))

    ground_leaf = model.part("ground_leaf")
    ground_leaf.visual(
        mesh_from_cadquery(
            _ground_leaf_shape(),
            "ground_leaf",
            tolerance=0.00025,
            angular_tolerance=0.03,
        ),
        material="ground_leaf_finish",
        name="ground_leaf_body",
    )

    hinge_pin = model.part("hinge_pin")
    hinge_pin.visual(
        Cylinder(radius=PIN_R, length=LEAF_W + 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material="pin_finish",
        name="pin_shaft",
    )
    hinge_pin.visual(
        Cylinder(radius=PIN_HEAD_R, length=PIN_HEAD_T),
        origin=Origin(
            xyz=(0.0, (LEAF_W / 2.0) + (PIN_HEAD_T / 2.0), 0.0),
            rpy=(-1.57079632679, 0.0, 0.0),
        ),
        material="pin_finish",
        name="upper_pin_head",
    )
    hinge_pin.visual(
        Cylinder(radius=PIN_HEAD_R, length=PIN_HEAD_T),
        origin=Origin(
            xyz=(0.0, -((LEAF_W / 2.0) + (PIN_HEAD_T / 2.0)), 0.0),
            rpy=(-1.57079632679, 0.0, 0.0),
        ),
        material="pin_finish",
        name="lower_pin_head",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(
            _moving_leaf_shape(),
            "moving_leaf",
            tolerance=0.0004,
            angular_tolerance=0.05,
        ),
        material="moving_leaf_finish",
        name="moving_leaf_body",
    )

    model.articulation(
        "ground_to_pin",
        ArticulationType.FIXED,
        parent=ground_leaf,
        child=hinge_pin,
        origin=Origin(),
    )
    model.articulation(
        "ground_to_leaf",
        ArticulationType.REVOLUTE,
        parent=ground_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_leaf = object_model.get_part("ground_leaf")
    hinge_pin = object_model.get_part("hinge_pin")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("ground_to_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        ground_leaf,
        hinge_pin,
        reason="through-pin is represented as a nominal no-clearance fit inside the grounded cheek bores",
    )
    ctx.allow_overlap(
        hinge_pin,
        moving_leaf,
        reason="nominal zero-clearance through-pin support is represented as a tight bearing fit",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "hinge_axis_matches_supported_pin",
        tuple(float(v) for v in hinge.axis) == (0.0, -1.0, 0.0),
        f"unexpected hinge axis: {hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            hinge_pin,
            moving_leaf,
            name="closed_leaf_is_supported_on_pin",
        )
        ctx.expect_contact(
            hinge_pin,
            ground_leaf,
            name="pin_is_mounted_in_grounded_clevis",
        )
        closed_aabb = ctx.part_world_aabb(moving_leaf)

    with ctx.pose({hinge: 1.25}):
        ctx.expect_contact(
            hinge_pin,
            moving_leaf,
            name="open_leaf_remains_supported",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_unintended_overlap")
        open_aabb = ctx.part_world_aabb(moving_leaf)

    if closed_aabb is None or open_aabb is None:
        ctx.fail("moving_leaf_aabb_available", "moving leaf AABB was unavailable")
    else:
        ctx.check(
            "leaf_opens_upward",
            open_aabb[1][2] > closed_aabb[1][2] + 0.020,
            f"closed max z={closed_aabb[1][2]:.4f}, open max z={open_aabb[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
