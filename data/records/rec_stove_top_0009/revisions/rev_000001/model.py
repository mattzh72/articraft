from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

COUNTER_WIDTH = 1.20
COUNTER_DEPTH = 0.70
COUNTER_THICKNESS = 0.04

COOKTOP_WIDTH = 0.75
COOKTOP_DEPTH = 0.50
COOKTOP_TOP_THICKNESS = 0.008
COOKTOP_TRAY_WIDTH = 0.698
COOKTOP_TRAY_DEPTH = 0.458
COOKTOP_TRAY_DEPTH_Z = 0.032

CUTOUT_WIDTH = 0.704
CUTOUT_DEPTH = 0.464

CONTROL_POD_WIDTH = 0.18
CONTROL_POD_DEPTH = 0.05
CONTROL_POD_HEIGHT = 0.072
CONTROL_POD_CENTER_Y = -0.165

KNOB_RADIUS = 0.016
KNOB_LENGTH = 0.018
KNOB_X_OFFSET = 0.0185
KNOB_Z_LOWER = 0.028
KNOB_Z_UPPER = 0.064

BURNER_X_OFFSET = 0.18
BURNER_FRONT_Y = -0.015
BURNER_REAR_Y = 0.12


def _offset_profile(
    profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_rect_plate_mesh(
    *, name: str, width: float, depth: float, height: float, radius: float
):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius=radius),
            height,
            center=True,
        ),
    )


def _rounded_rect_pod_mesh(
    *, name: str, width: float, height: float, depth: float, radius: float
):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius=radius),
            depth,
            center=True,
        ).rotate_x(math.pi * 0.5),
    )


def _add_burner(
    cooktop,
    *,
    index: int,
    center_x: float,
    center_y: float,
    grate_material,
    burner_material,
    cap_material,
) -> None:
    cooktop.visual(
        Cylinder(radius=0.065, length=0.003),
        origin=Origin(xyz=(center_x, center_y, COOKTOP_TOP_THICKNESS + 0.0015)),
        material=burner_material,
        name=f"burner_{index}_seat",
    )
    cooktop.visual(
        Cylinder(radius=0.046, length=0.005),
        origin=Origin(xyz=(center_x, center_y, COOKTOP_TOP_THICKNESS + 0.0025)),
        material=burner_material,
        name=f"burner_{index}_crown",
    )
    cooktop.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(center_x, center_y, COOKTOP_TOP_THICKNESS + 0.008)),
        material=cap_material,
        name=f"burner_{index}_cap",
    )

    grate_z = COOKTOP_TOP_THICKNESS + 0.016
    grate_height = 0.008
    cooktop.visual(
        Box((0.128, 0.014, grate_height)),
        origin=Origin(xyz=(center_x, center_y, grate_z)),
        material=grate_material,
        name=f"burner_{index}_grate_x",
    )
    cooktop.visual(
        Box((0.014, 0.128, grate_height)),
        origin=Origin(xyz=(center_x, center_y, grate_z)),
        material=grate_material,
        name=f"burner_{index}_grate_y",
    )

    foot_z = COOKTOP_TOP_THICKNESS + 0.006
    for foot_index, (dx, dy) in enumerate(
        ((0.050, 0.0), (-0.050, 0.0), (0.0, 0.050), (0.0, -0.050))
    ):
        cooktop.visual(
            Box((0.012, 0.012, 0.012)),
            origin=Origin(xyz=(center_x + dx, center_y + dy, foot_z)),
            material=grate_material,
            name=f"burner_{index}_foot_{foot_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_gas_cooktop", assets=ASSETS)

    countertop_stone = model.material("countertop_stone", rgba=(0.73, 0.72, 0.69, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_black = model.material("burner_black", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    marking_light = model.material("marking_light", rgba=(0.88, 0.88, 0.86, 1.0))

    countertop = model.part("countertop")
    counter_outer = rounded_rect_profile(COUNTER_WIDTH, COUNTER_DEPTH, radius=0.012)
    counter_cutout = _offset_profile(
        rounded_rect_profile(CUTOUT_WIDTH, CUTOUT_DEPTH, radius=0.010),
        0.0,
        0.0,
    )
    countertop_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            counter_outer,
            [counter_cutout],
            COUNTER_THICKNESS,
            center=True,
        ),
        ASSETS.mesh_path("countertop_ring.obj"),
    )
    countertop.visual(
        countertop_ring,
        origin=Origin(xyz=(0.0, 0.0, -COUNTER_THICKNESS * 0.5)),
        material=countertop_stone,
        name="countertop_ring",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -COUNTER_THICKNESS * 0.5)),
    )

    cooktop = model.part("cooktop")
    top_panel_mesh = _rounded_rect_plate_mesh(
        name="cooktop_top_panel.obj",
        width=COOKTOP_WIDTH,
        depth=COOKTOP_DEPTH,
        height=COOKTOP_TOP_THICKNESS,
        radius=0.026,
    )
    tray_mesh = _rounded_rect_plate_mesh(
        name="cooktop_insert_tray.obj",
        width=COOKTOP_TRAY_WIDTH,
        depth=COOKTOP_TRAY_DEPTH,
        height=COOKTOP_TRAY_DEPTH_Z,
        radius=0.020,
    )
    control_pod_mesh = _rounded_rect_pod_mesh(
        name="cooktop_control_pod.obj",
        width=CONTROL_POD_WIDTH,
        height=CONTROL_POD_HEIGHT,
        depth=CONTROL_POD_DEPTH,
        radius=0.016,
    )
    cooktop.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_TOP_THICKNESS * 0.5)),
        material=stainless,
        name="top_panel",
    )
    cooktop.visual(
        tray_mesh,
        origin=Origin(xyz=(0.0, 0.0, -COOKTOP_TRAY_DEPTH_Z * 0.5)),
        material=stainless,
        name="shallow_insert_tray",
    )
    cooktop.visual(
        control_pod_mesh,
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_POD_CENTER_Y,
                COOKTOP_TOP_THICKNESS + (CONTROL_POD_HEIGHT * 0.5),
            )
        ),
        material=stainless,
        name="control_pod",
    )

    burner_positions = (
        (-BURNER_X_OFFSET, BURNER_REAR_Y),
        (BURNER_X_OFFSET, BURNER_REAR_Y),
        (-BURNER_X_OFFSET, BURNER_FRONT_Y),
        (BURNER_X_OFFSET, BURNER_FRONT_Y),
    )
    for index, (burner_x, burner_y) in enumerate(burner_positions):
        _add_burner(
            cooktop,
            index=index,
            center_x=burner_x,
            center_y=burner_y,
            grate_material=dark_iron,
            burner_material=burner_black,
            cap_material=dark_iron,
        )

    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_TRAY_DEPTH_Z + 0.05)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(),
    )

    pod_front_y = CONTROL_POD_CENTER_Y - (CONTROL_POD_DEPTH * 0.5)
    knob_specs = (
        ("knob_lower_left", -KNOB_X_OFFSET, KNOB_Z_LOWER),
        ("knob_lower_right", KNOB_X_OFFSET, KNOB_Z_LOWER),
        ("knob_upper_left", -KNOB_X_OFFSET, KNOB_Z_UPPER),
        ("knob_upper_right", KNOB_X_OFFSET, KNOB_Z_UPPER),
    )
    for knob_name, knob_x, knob_z in knob_specs:
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            origin=Origin(
                xyz=(0.0, -KNOB_LENGTH * 0.5, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.002, 0.012)),
            origin=Origin(xyz=(0.0, -KNOB_LENGTH + 0.001, KNOB_RADIUS * 0.55)),
            material=marking_light,
            name="knob_indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_LENGTH),
            mass=0.09,
            origin=Origin(
                xyz=(0.0, -KNOB_LENGTH * 0.5, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
        )
        model.articulation(
            f"cooktop_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=cooktop,
            child=knob,
            origin=Origin(xyz=(knob_x, pod_front_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=7.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    knob_names = (
        "knob_lower_left",
        "knob_lower_right",
        "knob_upper_left",
        "knob_upper_right",
    )
    joint_names = tuple(f"cooktop_to_{name}" for name in knob_names)
    knobs = [object_model.get_part(name) for name in knob_names]
    joints = [object_model.get_articulation(name) for name in joint_names]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    ctx.check(
        "only_four_articulations",
        len(object_model.articulations) == 5,
        f"Expected exactly 5 articulations total (1 fixed cooktop mount + 4 knobs), found {len(object_model.articulations)}.",
    )

    ctx.expect_contact(cooktop, countertop, elem_a="top_panel", elem_b="countertop_ring")
    ctx.expect_gap(
        cooktop,
        countertop,
        axis="z",
        positive_elem="top_panel",
        negative_elem="countertop_ring",
        max_gap=0.0005,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        cooktop,
        countertop,
        axes="xy",
        elem_a="top_panel",
        elem_b="countertop_ring",
        min_overlap=0.02,
    )

    cooktop_aabb = ctx.part_world_aabb(cooktop)
    countertop_aabb = ctx.part_world_aabb(countertop)
    if cooktop_aabb is not None and countertop_aabb is not None:
        ctx.check(
            "cooktop_does_not_hang_below_counter",
            cooktop_aabb[0][2] >= countertop_aabb[0][2] - 1e-6,
            (
                f"Cooktop bottom z={cooktop_aabb[0][2]:.4f} must stay above "
                f"counter bottom z={countertop_aabb[0][2]:.4f}."
            ),
        )

    for burner_index in range(4):
        burner_aabb = ctx.part_element_world_aabb(cooktop, elem=f"burner_{burner_index}_cap")
        ctx.check(
            f"burner_{burner_index}_exists",
            burner_aabb is not None,
            f"Missing burner_{burner_index}_cap visual.",
        )

    for knob, joint, knob_name in zip(knobs, joints, knob_names):
        ctx.expect_contact(knob, cooktop, name=f"{knob_name}_mounted")
        ctx.check(
            f"{joint.name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{joint.name} must be CONTINUOUS, got {joint.articulation_type}.",
        )
        axis = joint.axis
        ctx.check(
            f"{joint.name}_front_to_back_axis",
            abs(axis[0]) < 1e-6 and abs(abs(axis[1]) - 1.0) < 1e-6 and abs(axis[2]) < 1e-6,
            f"{joint.name} axis must be front-to-back on Y, got {axis}.",
        )
        for pose_index, angle in enumerate((0.0, math.pi * 0.5, math.pi)):
            with ctx.pose({joint: angle}):
                ctx.expect_contact(
                    knob,
                    cooktop,
                    name=f"{knob_name}_contact_pose_{pose_index}",
                )

    lower_left = ctx.part_world_position("knob_lower_left")
    lower_right = ctx.part_world_position("knob_lower_right")
    upper_left = ctx.part_world_position("knob_upper_left")
    upper_right = ctx.part_world_position("knob_upper_right")
    if (
        lower_left is not None
        and lower_right is not None
        and upper_left is not None
        and upper_right is not None
    ):
        x_pitch = lower_right[0] - lower_left[0]
        z_pitch = upper_left[2] - lower_left[2]
        cluster_center_x = (lower_left[0] + lower_right[0] + upper_left[0] + upper_right[0]) * 0.25
        cluster_front_y = (lower_left[1] + lower_right[1] + upper_left[1] + upper_right[1]) * 0.25
        ctx.check(
            "knob_cluster_compact_square",
            0.03 <= x_pitch <= 0.05 and 0.03 <= z_pitch <= 0.05 and abs(x_pitch - z_pitch) <= 0.01,
            f"Knob cluster should be a compact square; pitches were x={x_pitch:.4f}, z={z_pitch:.4f}.",
        )
        ctx.check(
            "knob_cluster_front_centered",
            abs(cluster_center_x) <= 0.01 and cluster_front_y < -0.15,
            f"Knob cluster center should be near front-center; got ({cluster_center_x:.4f}, {cluster_front_y:.4f}).",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
