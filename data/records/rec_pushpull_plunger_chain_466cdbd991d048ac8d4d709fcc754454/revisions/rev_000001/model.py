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


BASE_L = 0.22
BASE_W = 0.14
BASE_T = 0.012

CORE_AXIS_Z = 0.03
CORE_REAR_X = -0.07

HINGE_X = 0.045
HINGE_Z = 0.062

CORE_TRAVEL = 0.035
PLATE_OPEN = 0.95


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_pushrod_assembly")

    model.material("body_finish", color=(0.28, 0.31, 0.35, 1.0))
    model.material("core_finish", color=(0.76, 0.78, 0.8, 1.0))
    model.material("plate_finish", color=(0.63, 0.66, 0.7, 1.0))

    body = model.part("body")
    body.visual(Box((BASE_L, BASE_W, BASE_T)), origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)), material="body_finish", name="base_plate")
    body.visual(Box((0.03, 0.018, 0.034)), origin=Origin(xyz=(-0.082, 0.024, 0.029)), material="body_finish", name="rear_left_post")
    body.visual(Box((0.03, 0.018, 0.034)), origin=Origin(xyz=(-0.082, -0.024, 0.029)), material="body_finish", name="rear_right_post")
    body.visual(Box((0.024, 0.066, 0.01)), origin=Origin(xyz=(-0.082, 0.0, 0.051)), material="body_finish", name="rear_bridge")
    body.visual(Box((0.115, 0.012, 0.024)), origin=Origin(xyz=(-0.02, 0.024, 0.024)), material="body_finish", name="left_guide_rail")
    body.visual(Box((0.115, 0.012, 0.024)), origin=Origin(xyz=(-0.02, -0.024, 0.024)), material="body_finish", name="right_guide_rail")
    body.visual(Box((0.02, 0.012, 0.03)), origin=Origin(xyz=(0.05, 0.018, 0.027)), material="body_finish", name="front_left_cheek")
    body.visual(Box((0.02, 0.012, 0.03)), origin=Origin(xyz=(0.05, -0.018, 0.027)), material="body_finish", name="front_right_cheek")
    body.visual(Box((0.07, 0.048, 0.008)), origin=Origin(xyz=(0.02, 0.0, 0.046)), material="body_finish", name="top_bridge")
    body.visual(Box((0.012, 0.006, 0.04)), origin=Origin(xyz=(HINGE_X, 0.012, 0.05)), material="body_finish", name="left_hinge_ear")
    body.visual(Box((0.012, 0.006, 0.04)), origin=Origin(xyz=(HINGE_X, -0.012, 0.05)), material="body_finish", name="right_hinge_ear")
    body.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.074)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    core = model.part("core")
    core.visual(
        Cylinder(radius=0.0075, length=0.12),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="core_finish",
        name="shaft",
    )
    core.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="core_finish",
        name="collar",
    )
    core.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.128, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="core_finish",
        name="tip",
    )
    core.visual(
        Box((0.028, 0.028, 0.018)),
        origin=Origin(xyz=(0.03, 0.0, -0.009)),
        material="core_finish",
        name="slider_shoe",
    )
    core.inertial = Inertial.from_geometry(
        Box((0.145, 0.024, 0.024)),
        mass=0.32,
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
    )

    plate = model.part("output_plate")
    plate.visual(
        Cylinder(radius=0.005, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="plate_finish",
        name="barrel",
    )
    plate.visual(
        Box((0.02, 0.018, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, 0.007)),
        material="plate_finish",
        name="arm",
    )
    plate.visual(
        Box((0.03, 0.042, 0.004)),
        origin=Origin(xyz=(0.034, 0.0, 0.01)),
        material="plate_finish",
        name="paddle",
    )
    plate.inertial = Inertial.from_geometry(
        Box((0.04, 0.046, 0.012)),
        mass=0.12,
        origin=Origin(xyz=(0.02, 0.0, 0.008)),
    )

    model.articulation(
        "body_to_core",
        ArticulationType.PRISMATIC,
        parent=body,
        child=core,
        origin=Origin(xyz=(CORE_REAR_X, 0.0, CORE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.08,
            lower=0.0,
            upper=CORE_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_output_plate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=plate,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=PLATE_OPEN,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    def part_present(name: str) -> bool:
        try:
            object_model.get_part(name)
            return True
        except Exception:
            return False

    def joint_present(name: str) -> bool:
        try:
            object_model.get_articulation(name)
            return True
        except Exception:
            return False

    expected_parts = ("body", "core", "output_plate")
    expected_joints = ("body_to_core", "body_to_output_plate")
    missing_parts = [name for name in expected_parts if not part_present(name)]
    missing_joints = [name for name in expected_joints if not joint_present(name)]

    ctx.check(
        "expected_parts_present",
        not missing_parts,
        details=f"missing parts: {missing_parts}",
    )
    ctx.check(
        "expected_articulations_present",
        not missing_joints,
        details=f"missing articulations: {missing_joints}",
    )

    body = object_model.get_part("body")
    core = object_model.get_part("core")
    plate = object_model.get_part("output_plate")
    slide = object_model.get_articulation("body_to_core")
    hinge = object_model.get_articulation("body_to_output_plate")

    ctx.expect_contact(body, core, contact_tol=0.001, name="core_is_seated_in_body")
    ctx.expect_contact(body, plate, contact_tol=0.001, name="plate_is_supported_by_body")
    ctx.expect_overlap(core, body, axes="yz", min_overlap=0.02, name="core_stays_guided_in_body")
    ctx.expect_within(plate, body, axes="xy", margin=0.004, name="closed_plate_stays_over_body_top")

    core_closed = ctx.part_world_position(core)
    with ctx.pose({slide: CORE_TRAVEL}):
        core_extended = ctx.part_world_position(core)
        ctx.expect_overlap(core, body, axes="yz", min_overlap=0.02, name="extended_core_remains_aligned")
    if core_closed is not None and core_extended is not None:
        dx = core_extended[0] - core_closed[0]
        dy = abs(core_extended[1] - core_closed[1])
        dz = abs(core_extended[2] - core_closed[2])
        ctx.check(
            "core_prismatic_motion_direction",
            dx > 0.03 and dy < 1e-4 and dz < 1e-4,
            details=f"expected +X slide without drift, got dx={dx:.5f}, dy={dy:.5f}, dz={dz:.5f}",
        )
    else:
        ctx.fail("core_prismatic_motion_direction", "could not resolve core world positions")

    plate_closed = ctx.part_element_world_aabb(plate, elem="paddle")
    with ctx.pose({hinge: PLATE_OPEN}):
        plate_open = ctx.part_element_world_aabb(plate, elem="paddle")
    if plate_closed is not None and plate_open is not None:
        closed_center_z = 0.5 * (plate_closed[0][2] + plate_closed[1][2])
        open_center_z = 0.5 * (plate_open[0][2] + plate_open[1][2])
        ctx.check(
            "output_plate_opens_upward",
            open_center_z > closed_center_z + 0.01,
            details=(
                "expected revolute plate to lift upward; "
                f"closed_center_z={closed_center_z:.5f}, open_center_z={open_center_z:.5f}"
            ),
        )
    else:
        ctx.fail("output_plate_opens_upward", "could not resolve output plate AABBs")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
