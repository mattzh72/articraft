from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


MAST_BASE_X = 0.42
MAST_BASE_Y = 0.28
MAST_BASE_Z = 0.08
RAIL_X = 0.055
RAIL_Y = 0.09
RAIL_Z = 1.20
RAIL_OFFSET_X = 0.135
BACK_WEB_X = 0.31
BACK_WEB_Y = 0.014
BACK_WEB_Z = 1.10
TOP_CAP_X = 0.36
TOP_CAP_Y = 0.12
TOP_CAP_Z = 0.06

CARRIAGE_PAD_X = 0.050
CARRIAGE_PAD_Y = 0.035
CARRIAGE_PAD_Z = 0.24
CARRIAGE_PAD_Y_CENTER = 0.0625
CARRIAGE_FRONT_X = 0.33
CARRIAGE_FRONT_Y = 0.036
CARRIAGE_FRONT_Z = 0.205
CARRIAGE_FRONT_Y_CENTER = 0.081
CARRIAGE_CHEEK_X = 0.035
CARRIAGE_CHEEK_Y = 0.065
CARRIAGE_CHEEK_Z = 0.13
CARRIAGE_CHEEK_OFFSET_X = 0.105
CARRIAGE_CHEEK_Y_CENTER = 0.113
CARRIAGE_CHEEK_Z_CENTER = 0.020
HINGE_EAR_X = 0.032
HINGE_EAR_Y = 0.022
HINGE_EAR_Z = 0.10
HINGE_EAR_OFFSET_X = 0.105
HINGE_EAR_Y_CENTER = 0.154
HINGE_EAR_Z_CENTER = 0.030

LIFT_HOME_Z = 0.34
LIFT_TRAVEL = 0.62

WRIST_HINGE_Y = HINGE_EAR_Y_CENTER
WRIST_HINGE_Z = 0.065
PLATE_PANEL_X = 0.28
PLATE_PANEL_Y = 0.018
PLATE_PANEL_Z = 0.22
PLATE_PANEL_Y_CENTER = 0.035
PLATE_PANEL_Z_CENTER = -(PLATE_PANEL_Z / 2.0)
PLATE_RIB_X = 0.022
PLATE_RIB_Y = 0.026
PLATE_RIB_Z = 0.10
PLATE_RIB_OFFSET_X = 0.075
PLATE_RIB_Y_CENTER = 0.018
PLATE_RIB_Z_CENTER = -0.050
PLATE_BRIDGE_X = 0.11
PLATE_BRIDGE_Y = 0.028
PLATE_BRIDGE_Z = 0.030
PLATE_BRIDGE_Y_CENTER = 0.014
PLATE_BRIDGE_Z_CENTER = -0.008
HINGE_BARREL_RADIUS = 0.013
HINGE_BARREL_LENGTH = 2.0 * (HINGE_EAR_OFFSET_X - (HINGE_EAR_X / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_lift_slide_with_wrist_plate")

    model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("carriage_orange", rgba=(0.86, 0.47, 0.13, 1.0))
    model.material("plate_gray", rgba=(0.77, 0.79, 0.81, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((MAST_BASE_X, MAST_BASE_Y, MAST_BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_Z / 2.0)),
        material="painted_steel",
        name="mast_base",
    )
    mast.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        origin=Origin(xyz=(-RAIL_OFFSET_X, 0.0, MAST_BASE_Z + (RAIL_Z / 2.0))),
        material="rail_steel",
        name="left_rail",
    )
    mast.visual(
        Box((RAIL_X, RAIL_Y, RAIL_Z)),
        origin=Origin(xyz=(RAIL_OFFSET_X, 0.0, MAST_BASE_Z + (RAIL_Z / 2.0))),
        material="rail_steel",
        name="right_rail",
    )
    mast.visual(
        Box((BACK_WEB_X, BACK_WEB_Y, BACK_WEB_Z)),
        origin=Origin(xyz=(0.0, -0.052, MAST_BASE_Z + (BACK_WEB_Z / 2.0))),
        material="painted_steel",
        name="back_web",
    )
    mast.visual(
        Box((0.30, 0.10, 0.07)),
        origin=Origin(xyz=(0.0, -0.005, 0.12)),
        material="painted_steel",
        name="lower_bridge",
    )
    mast.visual(
        Box((TOP_CAP_X, TOP_CAP_Y, TOP_CAP_Z)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_Z + RAIL_Z + (TOP_CAP_Z / 2.0))),
        material="painted_steel",
        name="top_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((MAST_BASE_X, MAST_BASE_Y, MAST_BASE_Z + RAIL_Z + TOP_CAP_Z)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, (MAST_BASE_Z + RAIL_Z + TOP_CAP_Z) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_PAD_X, CARRIAGE_PAD_Y, CARRIAGE_PAD_Z)),
        origin=Origin(xyz=(-RAIL_OFFSET_X, CARRIAGE_PAD_Y_CENTER, 0.0)),
        material="carriage_orange",
        name="left_pad",
    )
    carriage.visual(
        Box((CARRIAGE_PAD_X, CARRIAGE_PAD_Y, CARRIAGE_PAD_Z)),
        origin=Origin(xyz=(RAIL_OFFSET_X, CARRIAGE_PAD_Y_CENTER, 0.0)),
        material="carriage_orange",
        name="right_pad",
    )
    carriage.visual(
        Box((CARRIAGE_FRONT_X, CARRIAGE_FRONT_Y, CARRIAGE_FRONT_Z)),
        origin=Origin(xyz=(0.0, CARRIAGE_FRONT_Y_CENTER, 0.0)),
        material="carriage_orange",
        name="front_crosshead",
    )
    carriage.visual(
        Box((CARRIAGE_CHEEK_X, CARRIAGE_CHEEK_Y, CARRIAGE_CHEEK_Z)),
        origin=Origin(
            xyz=(-CARRIAGE_CHEEK_OFFSET_X, CARRIAGE_CHEEK_Y_CENTER, CARRIAGE_CHEEK_Z_CENTER)
        ),
        material="carriage_orange",
        name="left_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_CHEEK_X, CARRIAGE_CHEEK_Y, CARRIAGE_CHEEK_Z)),
        origin=Origin(
            xyz=(CARRIAGE_CHEEK_OFFSET_X, CARRIAGE_CHEEK_Y_CENTER, CARRIAGE_CHEEK_Z_CENTER)
        ),
        material="carriage_orange",
        name="right_cheek",
    )
    carriage.visual(
        Box((HINGE_EAR_X, HINGE_EAR_Y, HINGE_EAR_Z)),
        origin=Origin(xyz=(-HINGE_EAR_OFFSET_X, HINGE_EAR_Y_CENTER, HINGE_EAR_Z_CENTER)),
        material="carriage_orange",
        name="left_ear",
    )
    carriage.visual(
        Box((HINGE_EAR_X, HINGE_EAR_Y, HINGE_EAR_Z)),
        origin=Origin(xyz=(HINGE_EAR_OFFSET_X, HINGE_EAR_Y_CENTER, HINGE_EAR_Z_CENTER)),
        material="carriage_orange",
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_FRONT_X, 0.18, CARRIAGE_PAD_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
    )

    plate = model.part("wrist_plate")
    plate.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="plate_gray",
        name="hinge_barrel",
    )
    plate.visual(
        Box((PLATE_BRIDGE_X, PLATE_BRIDGE_Y, PLATE_BRIDGE_Z)),
        origin=Origin(xyz=(0.0, PLATE_BRIDGE_Y_CENTER, PLATE_BRIDGE_Z_CENTER)),
        material="plate_gray",
        name="hinge_bridge",
    )
    plate.visual(
        Box((PLATE_PANEL_X, PLATE_PANEL_Y, PLATE_PANEL_Z)),
        origin=Origin(xyz=(0.0, PLATE_PANEL_Y_CENTER, PLATE_PANEL_Z_CENTER)),
        material="plate_gray",
        name="plate_panel",
    )
    plate.visual(
        Box((PLATE_RIB_X, PLATE_RIB_Y, PLATE_RIB_Z)),
        origin=Origin(xyz=(-PLATE_RIB_OFFSET_X, PLATE_RIB_Y_CENTER, PLATE_RIB_Z_CENTER)),
        material="plate_gray",
        name="left_rib",
    )
    plate.visual(
        Box((PLATE_RIB_X, PLATE_RIB_Y, PLATE_RIB_Z)),
        origin=Origin(xyz=(PLATE_RIB_OFFSET_X, PLATE_RIB_Y_CENTER, PLATE_RIB_Z_CENTER)),
        material="plate_gray",
        name="right_rib",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_PANEL_X, 0.06, PLATE_PANEL_Z)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.02, -0.10)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=950.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "carriage_to_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=plate,
        origin=Origin(xyz=(0.0, WRIST_HINGE_Y, WRIST_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.95,
            effort=80.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    plate = object_model.get_part("wrist_plate")
    lift = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_plate")

    ctx.check("vertical lift axis", lift.axis == (0.0, 0.0, 1.0), details=f"axis={lift.axis}")
    ctx.check("horizontal wrist axis", wrist.axis == (1.0, 0.0, 0.0), details=f"axis={wrist.axis}")
    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        min_overlap=0.24,
        name="carriage stays laterally aligned to mast",
    )

    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    wrist_upper = wrist.motion_limits.upper if wrist.motion_limits is not None else None

    rest_carriage_pos = ctx.part_world_position(carriage)
    plate_aabb_rest = ctx.part_world_aabb(plate)

    with ctx.pose({lift: lift_upper}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="x",
            min_overlap=0.24,
            name="raised carriage remains aligned to mast",
        )

    ctx.check(
        "carriage rises on prismatic joint",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and lift_upper is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + (0.8 * lift_upper),
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}, travel={lift_upper}",
    )

    with ctx.pose({wrist: wrist_upper}):
        plate_aabb_open = ctx.part_world_aabb(plate)

    rest_center_y = None
    open_center_y = None
    if plate_aabb_rest is not None:
        rest_center_y = 0.5 * (plate_aabb_rest[0][1] + plate_aabb_rest[1][1])
    if plate_aabb_open is not None:
        open_center_y = 0.5 * (plate_aabb_open[0][1] + plate_aabb_open[1][1])

    ctx.check(
        "wrist plate swings forward",
        rest_center_y is not None
        and open_center_y is not None
        and open_center_y > rest_center_y + 0.04,
        details=f"rest_center_y={rest_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
