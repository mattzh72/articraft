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


BASE_DEPTH = 0.34
BASE_WIDTH = 0.46
BASE_THICKNESS = 0.045

COLUMN_DEPTH = 0.10
COLUMN_WIDTH = 0.18
COLUMN_HEIGHT = 1.24

GUIDE_DEPTH = 0.018
GUIDE_WIDTH = 0.034
GUIDE_HEIGHT = 0.94
GUIDE_CENTER_X = 0.054
GUIDE_CENTER_Y = 0.070
GUIDE_CENTER_Z = 0.66

ACTUATOR_COVER_DEPTH = 0.026
ACTUATOR_COVER_WIDTH = 0.042
ACTUATOR_COVER_HEIGHT = 1.02
ACTUATOR_COVER_CENTER_X = 0.032
ACTUATOR_COVER_CENTER_Z = 0.66

TOP_HEAD_DEPTH = 0.16
TOP_HEAD_WIDTH = 0.22
TOP_HEAD_HEIGHT = 0.08
TOP_HEAD_CENTER_Z = BASE_THICKNESS + COLUMN_HEIGHT - 0.01

PLATEN_BODY_DEPTH = 0.064
PLATEN_WIDTH = 0.30
PLATEN_HEIGHT = 0.24
SHOE_DEPTH = 0.018
SHOE_WIDTH = 0.050
SHOE_HEIGHT = 0.18
SHOE_CENTER_Y = GUIDE_CENTER_Y
PLATEN_BODY_CENTER_X = SHOE_DEPTH + (PLATEN_BODY_DEPTH / 2.0)

YOKE_DEPTH = 0.024
YOKE_WIDTH = 0.018
YOKE_HEIGHT = 0.12
YOKE_CENTER_Y = 0.079

END_PLATE_THICKNESS = 0.020
END_PLATE_WIDTH = 0.22
END_PLATE_HEIGHT = 0.18
HINGE_BARREL_RADIUS = 0.010
HINGE_BARREL_LENGTH = 0.14
END_PLATE_SPINE_DEPTH = 0.036
END_PLATE_SPINE_WIDTH = 0.080
END_PLATE_SPINE_HEIGHT = 0.10
END_PLATE_PANEL_CENTER_X = 0.046
END_PLATE_RIB_DEPTH = 0.028
END_PLATE_RIB_WIDTH = 0.10
END_PLATE_RIB_HEIGHT = 0.06
END_PLATE_RIB_CENTER_X = 0.028
END_PLATE_RIB_CENTER_Z = -0.045

LIFT_HOME_Z = 0.46
LIFT_TRAVEL = 0.56
LIFT_ORIGIN_X = GUIDE_CENTER_X + (GUIDE_DEPTH / 2.0)

WRIST_ORIGIN_X = 0.094
WRIST_LOWER = -0.45
WRIST_UPPER = 0.75


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return (
        maxs[0] - mins[0],
        maxs[1] - mins[1],
        maxs[2] - mins[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_lift_module")

    model.material("mast_graphite", rgba=(0.25, 0.28, 0.30, 1.0))
    model.material("platen_orange", rgba=(0.86, 0.49, 0.15, 1.0))
    model.material("face_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="mast_graphite",
        name="mast_base",
    )
    mast.visual(
        Box((COLUMN_DEPTH, COLUMN_WIDTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-0.005, 0.0, BASE_THICKNESS + (COLUMN_HEIGHT / 2.0))),
        material="mast_graphite",
        name="mast_body",
    )
    mast.visual(
        Box((GUIDE_DEPTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material="face_steel",
        name="guide_left",
    )
    mast.visual(
        Box((GUIDE_DEPTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, -GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material="face_steel",
        name="guide_right",
    )
    mast.visual(
        Box((ACTUATOR_COVER_DEPTH, ACTUATOR_COVER_WIDTH, ACTUATOR_COVER_HEIGHT)),
        origin=Origin(xyz=(ACTUATOR_COVER_CENTER_X, 0.0, ACTUATOR_COVER_CENTER_Z)),
        material="face_steel",
        name="actuator_cover",
    )
    mast.visual(
        Box((TOP_HEAD_DEPTH, TOP_HEAD_WIDTH, TOP_HEAD_HEIGHT)),
        origin=Origin(xyz=(-0.005, 0.0, TOP_HEAD_CENTER_Z)),
        material="mast_graphite",
        name="top_head",
    )
    mast.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS + COLUMN_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + COLUMN_HEIGHT) / 2.0)),
    )

    platen = model.part("platen")
    platen.visual(
        Box((PLATEN_BODY_DEPTH, PLATEN_WIDTH, PLATEN_HEIGHT)),
        origin=Origin(xyz=(PLATEN_BODY_CENTER_X, 0.0, 0.0)),
        material="platen_orange",
        name="platen_body",
    )
    platen.visual(
        Box((SHOE_DEPTH, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(SHOE_DEPTH / 2.0, SHOE_CENTER_Y, 0.0)),
        material="face_steel",
        name="shoe_left",
    )
    platen.visual(
        Box((SHOE_DEPTH, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(SHOE_DEPTH / 2.0, -SHOE_CENTER_Y, 0.0)),
        material="face_steel",
        name="shoe_right",
    )
    platen.visual(
        Box((YOKE_DEPTH, YOKE_WIDTH, YOKE_HEIGHT)),
        origin=Origin(xyz=(WRIST_ORIGIN_X - (YOKE_DEPTH / 2.0), YOKE_CENTER_Y, 0.0)),
        material="face_steel",
        name="yoke_left",
    )
    platen.visual(
        Box((YOKE_DEPTH, YOKE_WIDTH, YOKE_HEIGHT)),
        origin=Origin(xyz=(WRIST_ORIGIN_X - (YOKE_DEPTH / 2.0), -YOKE_CENTER_Y, 0.0)),
        material="face_steel",
        name="yoke_right",
    )
    platen.inertial = Inertial.from_geometry(
        Box((WRIST_ORIGIN_X, PLATEN_WIDTH, PLATEN_HEIGHT)),
        mass=8.5,
        origin=Origin(xyz=(WRIST_ORIGIN_X / 2.0, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="face_steel",
        name="hinge_barrel",
    )
    end_plate.visual(
        Box((END_PLATE_SPINE_DEPTH, END_PLATE_SPINE_WIDTH, END_PLATE_SPINE_HEIGHT)),
        origin=Origin(xyz=(END_PLATE_SPINE_DEPTH / 2.0, 0.0, 0.0)),
        material="face_steel",
        name="hinge_spine",
    )
    end_plate.visual(
        Box((END_PLATE_THICKNESS, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        origin=Origin(xyz=(END_PLATE_PANEL_CENTER_X, 0.0, 0.0)),
        material="face_steel",
        name="end_plate_face",
    )
    end_plate.visual(
        Box((END_PLATE_RIB_DEPTH, END_PLATE_RIB_WIDTH, END_PLATE_RIB_HEIGHT)),
        origin=Origin(xyz=(END_PLATE_RIB_CENTER_X, 0.0, END_PLATE_RIB_CENTER_Z)),
        material="face_steel",
        name="lower_rib",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((0.06, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    model.articulation(
        "mast_to_platen",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=platen,
        origin=Origin(xyz=(LIFT_ORIGIN_X, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=1600.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "platen_to_end_plate",
        ArticulationType.REVOLUTE,
        parent=platen,
        child=end_plate,
        origin=Origin(xyz=(WRIST_ORIGIN_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=WRIST_LOWER,
            upper=WRIST_UPPER,
            effort=60.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    platen = object_model.get_part("platen")
    end_plate = object_model.get_part("end_plate")
    lift = object_model.get_articulation("mast_to_platen")
    wrist = object_model.get_articulation("platen_to_end_plate")

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

    ctx.expect_contact(
        platen,
        mast,
        elem_a="shoe_left",
        elem_b="guide_left",
        name="left carriage shoe rides on the left mast guide",
    )
    ctx.expect_contact(
        platen,
        mast,
        elem_a="shoe_right",
        elem_b="guide_right",
        name="right carriage shoe rides on the right mast guide",
    )
    ctx.expect_contact(
        end_plate,
        platen,
        elem_a="hinge_barrel",
        elem_b="yoke_left",
        name="wrist barrel is captured by the left carriage yoke",
    )
    ctx.expect_contact(
        end_plate,
        platen,
        elem_a="hinge_barrel",
        elem_b="yoke_right",
        name="wrist barrel is captured by the right carriage yoke",
    )
    ctx.expect_overlap(
        platen,
        mast,
        axes="yz",
        min_overlap=0.16,
        elem_a="platen_body",
        elem_b="mast_body",
        name="platen overlaps the mast guide zone",
    )
    ctx.expect_overlap(
        end_plate,
        platen,
        axes="yz",
        min_overlap=0.14,
        elem_a="end_plate_face",
        elem_b="platen_body",
        name="end plate stays centered on the carriage face",
    )

    platen_box = ctx.part_world_aabb(platen)
    end_plate_box = ctx.part_world_aabb(end_plate)
    if platen_box is not None and end_plate_box is not None:
        platen_size = _aabb_size(platen_box)
        end_plate_size = _aabb_size(end_plate_box)
        ctx.check(
            "end plate is clearly smaller than the moving platen",
            (end_plate_size[1] < platen_size[1] - 0.05)
            and (end_plate_size[2] < platen_size[2] - 0.04),
            details=(
                f"platen size yz=({platen_size[1]:.3f}, {platen_size[2]:.3f}), "
                f"end plate yz=({end_plate_size[1]:.3f}, {end_plate_size[2]:.3f})"
            ),
        )

    with ctx.pose({lift: 0.0}):
        low_pos = ctx.part_world_position(platen)
    with ctx.pose({lift: LIFT_TRAVEL}):
        high_pos = ctx.part_world_position(platen)
    if low_pos is not None and high_pos is not None:
        dx = high_pos[0] - low_pos[0]
        dy = high_pos[1] - low_pos[1]
        dz = high_pos[2] - low_pos[2]
        ctx.check(
            "lift articulation raises the platen vertically",
            abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dz - LIFT_TRAVEL) < 1e-6,
            details=f"delta=({dx:.6f}, {dy:.6f}, {dz:.6f})",
        )

    with ctx.pose({wrist: 0.0}):
        face_closed = ctx.part_element_world_aabb(end_plate, elem="end_plate_face")
    with ctx.pose({wrist: 0.55}):
        face_tilted = ctx.part_element_world_aabb(end_plate, elem="end_plate_face")
    if face_closed is not None and face_tilted is not None:
        closed_max_x = face_closed[1][0]
        tilted_max_x = face_tilted[1][0]
        ctx.check(
            "wrist articulation pitches the face forward",
            tilted_max_x > closed_max_x + 0.035,
            details=(
                f"closed max x={closed_max_x:.3f}, "
                f"tilted max x={tilted_max_x:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
