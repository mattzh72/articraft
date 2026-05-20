from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BACK_PLATE_THICKNESS = 0.018
BACK_PLATE_CENTER_X = -0.025
BACK_PLATE_WIDTH = 0.090
BACK_PLATE_HEIGHT = 0.082
BACK_PLATE_HOLE_RADIUS = 0.0045
BACK_PLATE_HOLE_Y = 0.026
BACK_PLATE_HOLE_Z = 0.022

CHEEK_THICKNESS = 0.012
CHEEK_CENTER_Y = 0.012
CHEEK_BACK_LENGTH = 0.024
CHEEK_HEIGHT = 0.050
CHEEK_NOSE_RADIUS = 0.013
CHEEK_BORE_RADIUS = 0.0036

INNER_GAP = 0.012
PIN_RADIUS = 0.0038
PIN_LENGTH = 2.0 * CHEEK_THICKNESS + INNER_GAP

PLATE_THICKNESS = 0.010
PLATE_HEIGHT = 0.048
PLATE_BODY_LENGTH = 0.108
PLATE_TIP_RADIUS = PLATE_HEIGHT / 2.0
BARREL_LENGTH = INNER_GAP
BARREL_OUTER_RADIUS = 0.0115
BARREL_INNER_RADIUS = 0.0051
PLATE_HOLE_RADIUS = 0.0055
PLATE_HOLE_X_POSITIONS = (0.044, 0.082)


def _back_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(BACK_PLATE_WIDTH, BACK_PLATE_HEIGHT)
        .extrude(BACK_PLATE_THICKNESS / 2.0, both=True)
        .translate((BACK_PLATE_CENTER_X, 0.0, 0.0))
    )
    hole_cutters = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (y_pos, z_pos)
                for y_pos in (-BACK_PLATE_HOLE_Y, BACK_PLATE_HOLE_Y)
                for z_pos in (-BACK_PLATE_HOLE_Z, BACK_PLATE_HOLE_Z)
            ]
        )
        .circle(BACK_PLATE_HOLE_RADIUS)
        .extrude(BACK_PLATE_THICKNESS, both=True)
        .translate((BACK_PLATE_CENTER_X, 0.0, 0.0))
    )
    return plate.cut(hole_cutters)


def _cheek_shape() -> cq.Workplane:
    cheek_blank = (
        cq.Workplane("XY")
        .box(
            CHEEK_BACK_LENGTH,
            CHEEK_THICKNESS,
            CHEEK_HEIGHT,
            centered=(False, True, True),
        )
        .translate((-CHEEK_BACK_LENGTH, 0.0, 0.0))
    )
    nose = cq.Workplane("XZ").circle(CHEEK_NOSE_RADIUS).extrude(CHEEK_THICKNESS / 2.0, both=True)
    bore = cq.Workplane("XZ").circle(CHEEK_BORE_RADIUS).extrude(CHEEK_THICKNESS, both=True)
    return cheek_blank.union(nose).cut(bore)


def _moving_plate_shape() -> cq.Workplane:
    plate_body = (
        cq.Workplane("XY")
        .box(
            PLATE_BODY_LENGTH,
            PLATE_THICKNESS,
            PLATE_HEIGHT,
            centered=(False, True, True),
        )
    )
    rounded_tip = (
        cq.Workplane("XZ")
        .center(PLATE_BODY_LENGTH, 0.0)
        .circle(PLATE_TIP_RADIUS)
        .extrude(PLATE_THICKNESS / 2.0, both=True)
    )
    barrel = cq.Workplane("XZ").circle(BARREL_OUTER_RADIUS).extrude(BARREL_LENGTH / 2.0, both=True)

    plate = plate_body.union(rounded_tip).union(barrel)

    for x_pos in PLATE_HOLE_X_POSITIONS:
        plate = plate.cut(
            cq.Workplane("XZ")
            .center(x_pos, 0.0)
            .circle(PLATE_HOLE_RADIUS)
            .extrude(PLATE_THICKNESS, both=True)
        )

    pin_clearance = (
        cq.Workplane("XZ")
        .circle(BARREL_INNER_RADIUS)
        .extrude((BARREL_LENGTH + 0.002) / 2.0, both=True)
    )
    return plate.cut(pin_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clevis_backed_hinge_unit")

    support_finish = model.material("support_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.74, 0.77, 0.80, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.86, 0.87, 0.89, 1.0))

    fixed_support = model.part("fixed_support")
    fixed_support.visual(
        mesh_from_cadquery(_back_plate_shape(), "clevis_back_plate"),
        material=support_finish,
        name="back_plate",
    )
    fixed_support.visual(
        mesh_from_cadquery(_cheek_shape(), "left_clevis_cheek"),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, 0.0)),
        material=support_finish,
        name="left_cheek",
    )
    fixed_support.visual(
        mesh_from_cadquery(_cheek_shape(), "right_clevis_cheek"),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, 0.0)),
        material=support_finish,
        name="right_cheek",
    )
    fixed_support.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="hinge_pin",
    )
    fixed_support.inertial = Inertial.from_geometry(
        Box((0.047, BACK_PLATE_WIDTH, BACK_PLATE_HEIGHT)),
        mass=1.25,
        origin=Origin(xyz=(-0.0105, 0.0, 0.0)),
    )

    moving_plate = model.part("moving_plate")
    moving_plate.visual(
        mesh_from_cadquery(_moving_plate_shape(), "moving_plate_leaf"),
        material=plate_finish,
        name="moving_plate_body",
    )
    moving_plate.inertial = Inertial.from_geometry(
        Box((0.140, PLATE_THICKNESS, PLATE_HEIGHT)),
        mass=0.42,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_plate",
        ArticulationType.REVOLUTE,
        parent=fixed_support,
        child=moving_plate,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.30,
            effort=14.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_support = object_model.get_part("fixed_support")
    moving_plate = object_model.get_part("moving_plate")
    hinge = object_model.get_articulation("support_to_plate")

    ctx.check(
        "hinge parts exist",
        fixed_support is not None and moving_plate is not None and hinge is not None,
        details="Expected fixed_support, moving_plate, and support_to_plate articulation.",
    )
    ctx.check(
        "hinge axis lifts the plate upward",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    ctx.expect_contact(
        fixed_support,
        moving_plate,
        elem_a="left_cheek",
        elem_b="moving_plate_body",
        contact_tol=0.0003,
        name="left cheek captures the hinge barrel",
    )
    ctx.expect_contact(
        fixed_support,
        moving_plate,
        elem_a="right_cheek",
        elem_b="moving_plate_body",
        contact_tol=0.0003,
        name="right cheek captures the hinge barrel",
    )
    ctx.expect_gap(
        moving_plate,
        fixed_support,
        axis="x",
        positive_elem="moving_plate_body",
        negative_elem="back_plate",
        min_gap=0.0030,
        name="moving plate clears support back plate",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))

    rest_aabb = ctx.part_element_world_aabb(moving_plate, elem="moving_plate_body")
    open_center = None
    rest_center = _aabb_center(rest_aabb)
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(moving_plate, elem="moving_plate_body")
        open_center = _aabb_center(open_aabb)
    ctx.check(
        "plate opens upward on the hinge axis",
        rest_center is not None
        and open_center is not None
        and open_center[2] > rest_center[2] + 0.025,
        details=f"rest_center={rest_center}, open_center={open_center}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
