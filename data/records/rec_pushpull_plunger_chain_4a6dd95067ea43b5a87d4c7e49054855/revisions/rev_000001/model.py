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


BASE_T = 0.012
BASE_L = 0.210
BASE_W = 0.110

HOUSING_L = 0.180
HOUSING_W = 0.080
HOUSING_H = 0.090

PLUNGER_AXIS_Z = 0.045
GUIDE_BORE_R = 0.0115
PLUNGER_TRAVEL = 0.022

PLUNGER_BUTTON_R = 0.016
PLUNGER_BUTTON_LEN = 0.040
PLUNGER_BUTTON_START = -0.075

PLUNGER_ROD_R = 0.010
PLUNGER_ROD_LEN = 0.183
PLUNGER_ROD_START = -0.035

PLUNGER_TIP_R = 0.0085
PLUNGER_TIP_LEN = 0.019
PLUNGER_TIP_START = PLUNGER_ROD_START + PLUNGER_ROD_LEN
PLUNGER_TIP_END = PLUNGER_TIP_START + PLUNGER_TIP_LEN

PLUNGER_COLLAR_R = 0.011
PLUNGER_COLLAR_LEN = 0.006
PLUNGER_COLLAR_START = -0.018

PLUNGER_STOP_R = 0.014
PLUNGER_STOP_LEN = 0.010
PLUNGER_STOP_START = -0.045

PLUNGER_REST_GAP = 0.0

HINGE_X = (HOUSING_L / 2.0) + 0.011
HINGE_Z = 0.028
HINGE_R = 0.0055
TAB_BARREL_R = 0.0048
TAB_BARREL_LEN = 0.012
HINGE_SIDE_GAP = 0.001
HINGE_EAR_X = 0.012
HINGE_EAR_Z = 0.014
TAB_W = 0.028
TAB_H = 0.032
TAB_T = 0.004
TAB_OPEN_ANGLE = 0.95
TAB_CONTACT_ANGLE = 0.55
PLUNGER_CONTACT_EXTENSION = 0.010

PLUNGER_JOINT_X = HINGE_X - PLUNGER_REST_GAP - PLUNGER_TIP_END

FRONT_RECESS_D = 0.028
FRONT_RECESS_W = 0.042
FRONT_RECESS_H = 0.040
FRONT_RECESS_Z = PLUNGER_AXIS_Z


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center_xyz[0], center_xyz[1] - (length / 2.0), center_xyz[2]))
    )


def _make_housing_shape() -> cq.Workplane:
    housing_body = cq.Workplane("XY").box(HOUSING_L, HOUSING_W, HOUSING_H).translate(
        (0.0, 0.0, BASE_T + (HOUSING_H / 2.0))
    )
    mounting_base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T).translate(
        (0.0, 0.0, BASE_T / 2.0)
    )
    shell = housing_body.union(mounting_base)

    guide_bore = (
        cq.Workplane("YZ")
        .circle(GUIDE_BORE_R)
        .extrude(HOUSING_L + 0.060)
        .translate((-(HOUSING_L / 2.0) - 0.030, 0.0, PLUNGER_AXIS_Z))
    )
    front_recess = cq.Workplane("XY").box(
        FRONT_RECESS_D,
        FRONT_RECESS_W,
        FRONT_RECESS_H,
    ).translate(
        (
            (HOUSING_L / 2.0) - (FRONT_RECESS_D / 2.0) + 0.001,
            0.0,
            FRONT_RECESS_Z,
        )
    )
    shell = shell.cut(guide_bore).cut(front_recess)

    ear_y_len = (HOUSING_W - TAB_BARREL_LEN - (2.0 * HINGE_SIDE_GAP)) / 2.0
    ear_x_center = (HOUSING_L / 2.0) + (HINGE_EAR_X / 2.0) - 0.001
    left_ear_y = -((TAB_BARREL_LEN / 2.0) + HINGE_SIDE_GAP + (ear_y_len / 2.0))
    right_ear_y = (TAB_BARREL_LEN / 2.0) + HINGE_SIDE_GAP + (ear_y_len / 2.0)

    left_ear = cq.Workplane("XY").box(HINGE_EAR_X, ear_y_len, HINGE_EAR_Z).translate(
        (ear_x_center, left_ear_y, HINGE_Z)
    )
    right_ear = cq.Workplane("XY").box(HINGE_EAR_X, ear_y_len, HINGE_EAR_Z).translate(
        (ear_x_center, right_ear_y, HINGE_Z)
    )

    return shell.union(left_ear).union(right_ear)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plunger_tab_mechanism")

    model.material("housing_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("plunger_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("tab_orange", rgba=(0.87, 0.48, 0.16, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shape(), "housing_shell"),
        material="housing_gray",
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + HOUSING_H)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + HOUSING_H) / 2.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=PLUNGER_BUTTON_R, length=PLUNGER_BUTTON_LEN),
        origin=Origin(
            xyz=(PLUNGER_BUTTON_START + (PLUNGER_BUTTON_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_button",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_ROD_R, length=PLUNGER_ROD_LEN),
        origin=Origin(
            xyz=(PLUNGER_ROD_START + (PLUNGER_ROD_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_COLLAR_R, length=PLUNGER_COLLAR_LEN),
        origin=Origin(
            xyz=(PLUNGER_COLLAR_START + (PLUNGER_COLLAR_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_collar",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_STOP_R, length=PLUNGER_STOP_LEN),
        origin=Origin(
            xyz=(PLUNGER_STOP_START + (PLUNGER_STOP_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_stop",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_TIP_R, length=PLUNGER_TIP_LEN),
        origin=Origin(
            xyz=(PLUNGER_TIP_START + (PLUNGER_TIP_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="plunger_steel",
        name="plunger_tip",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=PLUNGER_BUTTON_R, length=PLUNGER_TIP_END - PLUNGER_BUTTON_START),
        mass=0.24,
        origin=Origin(
            xyz=(
                (PLUNGER_BUTTON_START + PLUNGER_TIP_END) / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    tab = model.part("tab")
    tab.visual(
        Box((TAB_T, TAB_W, TAB_H)),
        origin=Origin(xyz=(TAB_T / 2.0, 0.0, TAB_H / 2.0)),
        material="tab_orange",
        name="tab_plate",
    )
    tab.visual(
        Cylinder(radius=TAB_BARREL_R, length=TAB_BARREL_LEN),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="tab_orange",
        name="tab_barrel",
    )
    tab.inertial = Inertial.from_geometry(
        Box((TAB_T, TAB_W, TAB_H)),
        mass=0.08,
        origin=Origin(xyz=(TAB_T / 2.0, 0.0, TAB_H / 2.0)),
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(PLUNGER_JOINT_X, 0.0, PLUNGER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLUNGER_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "housing_to_tab",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TAB_OPEN_ANGLE,
            effort=4.0,
            velocity=1.2,
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

    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("housing_to_plunger")
    hinge = object_model.get_articulation("housing_to_tab")

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_within(
            plunger,
            housing,
            axes="yz",
            inner_elem="plunger_rod",
            outer_elem="housing_shell",
            margin=0.002,
            name="plunger rod stays centered in the housing guide",
        )
        ctx.expect_contact(
            plunger,
            tab,
            elem_a="plunger_tip",
            elem_b="tab_plate",
            contact_tol=0.001,
            name="retracted plunger nose meets the closed tab",
        )
        tab_plate_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
        ctx.check(
            "closed tab mounts just ahead of the housing front",
            tab_plate_aabb is not None and abs(tab_plate_aabb[0][0] - HINGE_X) <= 0.001,
            details=f"tab_plate_aabb={tab_plate_aabb}",
        )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            plunger,
            housing,
            axes="yz",
            inner_elem="plunger_rod",
            outer_elem="housing_shell",
            margin=0.002,
            name="extended plunger rod stays aligned with the housing guide",
        )
        ctx.expect_overlap(
            plunger,
            housing,
            axes="x",
            elem_a="plunger_rod",
            elem_b="housing_shell",
            min_overlap=0.120,
            name="plunger keeps retained insertion at full extension",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger extends forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.020,
        details=f"rest_pos={rest_pos}, extended_pos={extended_pos}",
    )

    closed_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")
    with ctx.pose({hinge: TAB_OPEN_ANGLE}):
        open_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_plate")

    ctx.check(
        "tab swings outward",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][0] > closed_tab_aabb[1][0] + 0.018,
        details=f"closed_tab_aabb={closed_tab_aabb}, open_tab_aabb={open_tab_aabb}",
    )

    with ctx.pose({slide: PLUNGER_CONTACT_EXTENSION, hinge: TAB_CONTACT_ANGLE}):
        ctx.expect_contact(
            plunger,
            tab,
            elem_a="plunger_tip",
            elem_b="tab_plate",
            contact_tol=0.0015,
            name="extended plunger still bears on the opened tab",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
