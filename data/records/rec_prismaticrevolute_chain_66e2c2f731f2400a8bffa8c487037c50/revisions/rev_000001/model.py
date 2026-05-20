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


RAIL_LENGTH = 0.68
RAIL_BASE_WIDTH = 0.16
RAIL_BASE_THICK = 0.02
RAIL_GUIDE_LENGTH = 0.58
RAIL_GUIDE_WIDTH = 0.074
RAIL_GUIDE_HEIGHT = 0.045

SLIDE_CLEARANCE = 0.0
SLIDE_LOWER = -0.18
SLIDE_UPPER = 0.18

TRUCK_LENGTH = 0.19
TRUCK_WIDTH = 0.15
TRUCK_HEIGHT = 0.062
TRUCK_CHEEK_WIDTH = 0.036
TRUCK_BRIDGE_THICK = 0.012
TRUCK_PIVOT_X = (TRUCK_LENGTH / 2.0) - 0.020
TRUCK_PIVOT_Z = TRUCK_HEIGHT + 0.024

EAR_LENGTH = 0.022
EAR_THICKNESS = 0.012
EAR_HEIGHT = 0.036
PEDESTAL_LENGTH = 0.032
PEDESTAL_WIDTH = 0.054
PEDESTAL_HEIGHT = 0.008

NOSE_BARREL_RADIUS = 0.011
NOSE_BARREL_LENGTH = 0.076
EAR_CENTER_Y = (NOSE_BARREL_LENGTH / 2.0) + (EAR_THICKNESS / 2.0)
NOSE_BODY_LENGTH = 0.108
NOSE_BODY_WIDTH = 0.046
NOSE_BODY_HEIGHT = 0.028
NOSE_BODY_BOTTOM_Z = -0.014
NOSE_OPEN_UPPER = 1.05


def _make_nose_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            NOSE_BODY_LENGTH,
            NOSE_BODY_WIDTH,
            NOSE_BODY_HEIGHT,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, NOSE_BODY_BOTTOM_Z))
    )

    top_cutter = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.004, 0.006),
                (0.040, 0.006),
                (NOSE_BODY_LENGTH + 0.004, 0.020),
                (NOSE_BODY_LENGTH + 0.004, 0.050),
                (-0.004, 0.050),
            ]
        )
        .close()
        .extrude(NOSE_BODY_WIDTH + 0.012, both=True)
    )
    body = body.cut(top_cutter)

    bottom_cutter = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.004, -0.050),
                (NOSE_BODY_LENGTH + 0.004, -0.050),
                (NOSE_BODY_LENGTH + 0.004, -0.006),
                (0.048, -0.012),
                (-0.004, -0.023),
            ]
        )
        .close()
        .extrude(NOSE_BODY_WIDTH + 0.012, both=True)
    )
    body = body.cut(bottom_cutter)

    return body


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_slide_hinged_nose")

    model.material("rail_black", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("rail_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("truck_orange", rgba=(0.86, 0.47, 0.14, 1.0))
    model.material("nose_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("pin_steel", rgba=(0.75, 0.77, 0.80, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK / 2.0)),
        material="rail_black",
        name="rail_base",
    )
    rail.visual(
        Box((RAIL_GUIDE_LENGTH, RAIL_GUIDE_WIDTH, RAIL_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                RAIL_BASE_THICK + (RAIL_GUIDE_HEIGHT / 2.0),
            )
        ),
        material="rail_steel",
        name="rail_guide",
    )
    for x_pos in (-0.24, 0.24):
        for y_pos in (-0.052, 0.052):
            rail.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(
                    xyz=(x_pos, y_pos, RAIL_BASE_THICK + 0.002),
                ),
                material="pin_steel",
            )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_THICK + RAIL_GUIDE_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_BASE_THICK + RAIL_GUIDE_HEIGHT) / 2.0)),
    )

    truck = model.part("truck")
    truck.visual(
        Box((TRUCK_LENGTH, TRUCK_CHEEK_WIDTH, TRUCK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(TRUCK_WIDTH - TRUCK_CHEEK_WIDTH) / 2.0,
                TRUCK_HEIGHT / 2.0,
            )
        ),
        material="truck_orange",
        name="truck_left_cheek",
    )
    truck.visual(
        Box((TRUCK_LENGTH, TRUCK_CHEEK_WIDTH, TRUCK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (TRUCK_WIDTH - TRUCK_CHEEK_WIDTH) / 2.0,
                TRUCK_HEIGHT / 2.0,
            )
        ),
        material="truck_orange",
        name="truck_right_cheek",
    )
    truck.visual(
        Box((TRUCK_LENGTH, TRUCK_WIDTH, TRUCK_BRIDGE_THICK)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                TRUCK_HEIGHT - (TRUCK_BRIDGE_THICK / 2.0),
            )
        ),
        material="truck_orange",
        name="truck_bridge",
    )
    truck.visual(
        Box((PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                TRUCK_PIVOT_X - 0.012,
                0.0,
                TRUCK_HEIGHT + (PEDESTAL_HEIGHT / 2.0),
            )
        ),
        material="truck_orange",
        name="truck_pedestal",
    )
    truck.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                TRUCK_PIVOT_X,
                -EAR_CENTER_Y,
                TRUCK_HEIGHT + (EAR_HEIGHT / 2.0),
            )
        ),
        material="truck_orange",
        name="truck_left_ear",
    )
    truck.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                TRUCK_PIVOT_X,
                EAR_CENTER_Y,
                TRUCK_HEIGHT + (EAR_HEIGHT / 2.0),
            )
        ),
        material="truck_orange",
        name="truck_right_ear",
    )
    truck.inertial = Inertial.from_geometry(
        Box((TRUCK_LENGTH, TRUCK_WIDTH, TRUCK_HEIGHT + EAR_HEIGHT)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, (TRUCK_HEIGHT + EAR_HEIGHT) / 2.0)),
    )

    nose = model.part("nose")
    nose.visual(
        Cylinder(radius=NOSE_BARREL_RADIUS, length=NOSE_BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="nose_barrel",
    )
    nose.visual(
        mesh_from_cadquery(_make_nose_body_shape(), "nose_body"),
        material="nose_dark",
        name="nose_body",
    )
    nose.inertial = Inertial.from_geometry(
        Box((NOSE_BODY_LENGTH, NOSE_BODY_WIDTH, 0.036)),
        mass=0.45,
        origin=Origin(
            xyz=(NOSE_BODY_LENGTH / 2.0, 0.0, -0.008),
        ),
    )

    model.articulation(
        "rail_to_truck",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK + SLIDE_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=220.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "truck_to_nose",
        ArticulationType.REVOLUTE,
        parent=truck,
        child=nose,
        origin=Origin(xyz=(TRUCK_PIVOT_X, 0.0, TRUCK_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=NOSE_OPEN_UPPER,
            effort=18.0,
            velocity=2.0,
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
    rail = object_model.get_part("rail")
    truck = object_model.get_part("truck")
    nose = object_model.get_part("nose")
    slide = object_model.get_articulation("rail_to_truck")
    hinge = object_model.get_articulation("truck_to_nose")

    ctx.check(
        "required assembly present",
        rail is not None and truck is not None and nose is not None and slide is not None and hinge is not None,
        details="rail, truck, nose, and both articulations must exist",
    )
    ctx.expect_gap(
        truck,
        rail,
        axis="z",
        negative_elem="rail_base",
        min_gap=0.0,
        max_gap=0.0015,
        name="truck rides just above the grounded rail base",
    )
    ctx.expect_contact(
        nose,
        truck,
        elem_a="nose_barrel",
        elem_b="truck_left_ear",
        name="nose barrel is supported by the left hinge ear",
    )
    ctx.expect_contact(
        nose,
        truck,
        elem_a="nose_barrel",
        elem_b="truck_right_ear",
        name="nose barrel is supported by the right hinge ear",
    )

    rest_truck_pos = ctx.part_world_position(truck)
    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_within(
            truck,
            rail,
            axes="x",
            outer_elem="rail_base",
            margin=0.002,
            name="truck remains over the rail at full extension",
        )
        extended_truck_pos = ctx.part_world_position(truck)
    ctx.check(
        "truck translates forward on the rail",
        rest_truck_pos is not None
        and extended_truck_pos is not None
        and extended_truck_pos[0] > rest_truck_pos[0] + 0.12,
        details=f"rest={rest_truck_pos}, extended={extended_truck_pos}",
    )

    rest_nose_aabb = ctx.part_element_world_aabb(nose, elem="nose_body")
    with ctx.pose({hinge: NOSE_OPEN_UPPER}):
        ctx.expect_gap(
            nose,
            truck,
            axis="z",
            positive_elem="nose_body",
            negative_elem="truck_bridge",
            min_gap=0.008,
            name="opened nose clears the truck deck",
        )
        open_nose_aabb = ctx.part_element_world_aabb(nose, elem="nose_body")
    rest_nose_center = _aabb_center(rest_nose_aabb)
    open_nose_center = _aabb_center(open_nose_aabb)
    ctx.check(
        "nose pitches upward when opened",
        rest_nose_center is not None
        and open_nose_center is not None
        and open_nose_center[2] > rest_nose_center[2] + 0.025,
        details=f"rest={rest_nose_center}, open={open_nose_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
