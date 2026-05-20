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
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


FASCIA_OUTER_WIDTH = 0.480
FASCIA_OUTER_HEIGHT = 0.218
FASCIA_OPENING_WIDTH = 0.372
FASCIA_OPENING_HEIGHT = 0.154
FASCIA_THICKNESS = 0.018

TUB_FRONT_Y = 0.0
TUB_DEPTH = 0.195

DOOR_WIDTH = 0.392
DOOR_HEIGHT = 0.168
DOOR_HINGE_Y = 0.0045
DOOR_HINGE_Z = -0.118
DOOR_SHELL_Z_OFFSET = 0.032
HINGE_X_OFFSET = 0.177


def _section_xz(
    width: float,
    height: float,
    radius: float,
    *,
    y_pos: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y_pos, z_center + z) for x, z in rounded_rect_profile(width, height, radius)]


def _section_yz(
    thickness: float,
    height: float,
    radius: float,
    *,
    x_pos: float,
    y_center: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y_center + y, z_center + z) for y, z in rounded_rect_profile(thickness, height, radius)]


def _build_fascia_ring_mesh():
    ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(FASCIA_OUTER_WIDTH, FASCIA_OUTER_HEIGHT, radius=0.026, corner_segments=10),
        [rounded_rect_profile(FASCIA_OPENING_WIDTH, FASCIA_OPENING_HEIGHT, radius=0.014, corner_segments=8)],
        height=FASCIA_THICKNESS,
        center=True,
    )
    ring.rotate_x(-math.pi / 2.0)
    return ring


def _build_tub_shell_mesh():
    outer_spec = SectionLoftSpec(
        sections=(
            _section_xz(0.364, 0.148, 0.014, y_pos=TUB_FRONT_Y, z_center=-0.004),
            _section_xz(0.336, 0.136, 0.012, y_pos=-0.090, z_center=-0.009),
            _section_xz(0.302, 0.124, 0.010, y_pos=-TUB_DEPTH, z_center=-0.014),
        ),
        cap=True,
        solid=True,
    )
    inner_spec = SectionLoftSpec(
        sections=(
            _section_xz(0.342, 0.128, 0.010, y_pos=TUB_FRONT_Y, z_center=0.001),
            _section_xz(0.314, 0.116, 0.009, y_pos=-0.086, z_center=-0.004),
            _section_xz(0.284, 0.106, 0.008, y_pos=-(TUB_DEPTH - 0.007), z_center=-0.008),
        ),
        cap=True,
        solid=True,
    )
    outer = repair_loft(outer_spec)
    inner = repair_loft(inner_spec)
    return boolean_difference(outer, inner)


def _build_door_shell_mesh():
    outer_spec = SectionLoftSpec(
        sections=(
            _section_yz(0.026, DOOR_HEIGHT - 0.004, 0.012, x_pos=-0.196, y_center=0.018, z_center=DOOR_HEIGHT / 2.0),
            _section_yz(0.027, DOOR_HEIGHT - 0.002, 0.012, x_pos=-0.098, y_center=0.021, z_center=DOOR_HEIGHT / 2.0),
            _section_yz(0.028, DOOR_HEIGHT, 0.013, x_pos=0.0, y_center=0.0225, z_center=DOOR_HEIGHT / 2.0),
            _section_yz(0.027, DOOR_HEIGHT - 0.002, 0.012, x_pos=0.098, y_center=0.021, z_center=DOOR_HEIGHT / 2.0),
            _section_yz(0.026, DOOR_HEIGHT - 0.004, 0.012, x_pos=0.196, y_center=0.018, z_center=DOOR_HEIGHT / 2.0),
        ),
        cap=True,
        solid=True,
    )
    outer = repair_loft(outer_spec)
    inner_cavity = BoxGeometry((0.330, 0.032, 0.122)).translate(0.0, 0.011, 0.093)
    shell = boolean_difference(outer, inner_cavity)
    shell.translate(0.0, 0.0, DOOR_SHELL_Z_OFFSET)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_glove_box", assets=ASSETS)

    dash_plastic = model.material("dash_plastic", rgba=(0.28, 0.29, 0.31, 1.0))
    tub_plastic = model.material("tub_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    door_plastic = model.material("door_plastic", rgba=(0.36, 0.37, 0.40, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    latch_trim = model.material("latch_trim", rgba=(0.12, 0.12, 0.13, 1.0))

    dashboard_face = model.part("dashboard_face")
    fascia_ring_mesh = mesh_from_geometry(_build_fascia_ring_mesh(), ASSETS.mesh_path("glovebox_fascia_ring.obj"))
    dashboard_face.visual(fascia_ring_mesh, material=dash_plastic, name="fascia_ring")
    dashboard_face.visual(
        Box((0.540, 0.044, 0.040)),
        origin=Origin(xyz=(0.0, -0.008, 0.129)),
        material=dash_plastic,
        name="top_brow",
    )
    dashboard_face.visual(
        Box((0.036, 0.030, 0.236)),
        origin=Origin(xyz=(-0.222, -0.006, -0.008)),
        material=dash_plastic,
        name="left_side_cheek",
    )
    dashboard_face.visual(
        Box((0.036, 0.030, 0.236)),
        origin=Origin(xyz=(0.222, -0.006, -0.008)),
        material=dash_plastic,
        name="right_side_cheek",
    )
    dashboard_face.visual(
        Box((0.410, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.021, -0.126)),
        material=dash_plastic,
        name="lower_sill",
    )
    dashboard_face.visual(
        Cylinder(radius=0.007, length=0.056),
        origin=Origin(xyz=(-HINGE_X_OFFSET, DOOR_HINGE_Y, DOOR_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_hinge_pin",
    )
    dashboard_face.visual(
        Cylinder(radius=0.007, length=0.056),
        origin=Origin(xyz=(HINGE_X_OFFSET, DOOR_HINGE_Y, DOOR_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_hinge_pin",
    )
    dashboard_face.inertial = Inertial.from_geometry(
        Box((0.540, 0.044, 0.306)),
        mass=2.9,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )

    storage_tub = model.part("storage_tub")
    tub_shell_mesh = mesh_from_geometry(_build_tub_shell_mesh(), ASSETS.mesh_path("glovebox_storage_tub.obj"))
    storage_tub.visual(tub_shell_mesh, material=tub_plastic, name="tub_shell")
    storage_tub.inertial = Inertial.from_geometry(
        Box((0.364, TUB_DEPTH, 0.148)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -(TUB_DEPTH / 2.0), -0.004)),
    )
    model.articulation(
        "dashboard_to_storage_tub",
        ArticulationType.FIXED,
        parent=dashboard_face,
        child=storage_tub,
        origin=Origin(xyz=(0.0, -(FASCIA_THICKNESS / 2.0), 0.0)),
    )

    glovebox_door = model.part("glovebox_door")
    door_shell_mesh = mesh_from_geometry(_build_door_shell_mesh(), ASSETS.mesh_path("glovebox_door_shell.obj"))
    glovebox_door.visual(door_shell_mesh, material=door_plastic, name="door_shell")
    glovebox_door.visual(
        Box((0.016, 0.014, 0.032)),
        origin=Origin(xyz=(-0.177, 0.0105, 0.0240)),
        material=hinge_metal,
        name="left_hinge_arm",
    )
    glovebox_door.visual(
        Box((0.016, 0.014, 0.032)),
        origin=Origin(xyz=(0.177, 0.0105, 0.0240)),
        material=hinge_metal,
        name="right_hinge_arm",
    )
    glovebox_door.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    glovebox_door.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    glovebox_door.visual(
        Box((0.088, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.031, 0.127 + DOOR_SHELL_Z_OFFSET)),
        material=latch_trim,
        name="pull_latch",
    )
    glovebox_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.038, DOOR_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.020, DOOR_HEIGHT / 2.0)),
    )
    model.articulation(
        "dashboard_to_glovebox_door",
        ArticulationType.REVOLUTE,
        parent=dashboard_face,
        child=glovebox_door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    dashboard_face = object_model.get_part("dashboard_face")
    storage_tub = object_model.get_part("storage_tub")
    glovebox_door = object_model.get_part("glovebox_door")
    door_hinge = object_model.get_articulation("dashboard_to_glovebox_door")

    fascia_ring = dashboard_face.get_visual("fascia_ring")
    left_hinge_pin = dashboard_face.get_visual("left_hinge_pin")
    right_hinge_pin = dashboard_face.get_visual("right_hinge_pin")

    tub_shell = storage_tub.get_visual("tub_shell")

    door_shell = glovebox_door.get_visual("door_shell")
    left_hinge_arm = glovebox_door.get_visual("left_hinge_arm")
    right_hinge_arm = glovebox_door.get_visual("right_hinge_arm")
    left_hinge_barrel = glovebox_door.get_visual("left_hinge_barrel")
    right_hinge_barrel = glovebox_door.get_visual("right_hinge_barrel")
    pull_latch = glovebox_door.get_visual("pull_latch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        glovebox_door,
        dashboard_face,
        elem_a=left_hinge_barrel,
        elem_b=left_hinge_pin,
        reason="Left hinge barrel intentionally wraps the fixed lower pivot pin.",
    )
    ctx.allow_overlap(
        glovebox_door,
        dashboard_face,
        elem_a=right_hinge_barrel,
        elem_b=right_hinge_pin,
        reason="Right hinge barrel intentionally wraps the fixed lower pivot pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        dashboard_face,
        storage_tub,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        positive_elem=fascia_ring,
        negative_elem=tub_shell,
        name="storage_tub_seats_on_dashboard_rear_face",
    )
    ctx.expect_overlap(
        storage_tub,
        dashboard_face,
        axes="xz",
        min_overlap=0.120,
        elem_a=tub_shell,
        elem_b=fascia_ring,
        name="storage_tub_aligned_behind_fascia_opening",
    )
    ctx.expect_gap(
        glovebox_door,
        dashboard_face,
        axis="y",
        min_gap=0.0005,
        max_gap=0.0040,
        positive_elem=door_shell,
        negative_elem=fascia_ring,
        name="door_panel_sits_flush_with_fascia",
    )
    ctx.expect_overlap(
        glovebox_door,
        dashboard_face,
        axes="xz",
        min_overlap=0.150,
        elem_a=door_shell,
        elem_b=fascia_ring,
        name="door_covers_glovebox_opening",
    )
    ctx.expect_contact(
        glovebox_door,
        dashboard_face,
        elem_a=left_hinge_barrel,
        elem_b=left_hinge_pin,
        name="left_lower_hinge_is_seated",
    )
    ctx.expect_contact(
        glovebox_door,
        dashboard_face,
        elem_a=right_hinge_barrel,
        elem_b=right_hinge_pin,
        name="right_lower_hinge_is_seated",
    )
    ctx.expect_contact(
        glovebox_door,
        glovebox_door,
        elem_a=left_hinge_arm,
        elem_b=door_shell,
        name="left_hinge_arm_bonds_to_door_shell",
    )
    ctx.expect_contact(
        glovebox_door,
        glovebox_door,
        elem_a=right_hinge_arm,
        elem_b=door_shell,
        name="right_hinge_arm_bonds_to_door_shell",
    )
    ctx.expect_contact(
        glovebox_door,
        glovebox_door,
        elem_a=pull_latch,
        elem_b=door_shell,
        name="pull_latch_mounts_to_outer_door_panel",
    )

    rest_door_aabb = ctx.part_element_world_aabb(glovebox_door, elem="door_shell")
    ctx.check("door_shell_aabb_present_at_rest", rest_door_aabb is not None, "Door shell AABB missing at rest.")
    assert rest_door_aabb is not None
    with ctx.pose({door_hinge: math.radians(65.0)}):
        open_door_aabb = ctx.part_element_world_aabb(glovebox_door, elem="door_shell")
        ctx.check("door_shell_aabb_present_open", open_door_aabb is not None, "Door shell AABB missing when open.")
        assert open_door_aabb is not None
        ctx.expect_gap(
            glovebox_door,
            storage_tub,
            axis="y",
            min_gap=0.018,
            positive_elem=door_shell,
            negative_elem=tub_shell,
            name="opened_door_swings_clear_of_storage_tub",
        )
        ctx.expect_contact(
            glovebox_door,
            dashboard_face,
            elem_a=left_hinge_barrel,
            elem_b=left_hinge_pin,
            name="left_hinge_contact_persists_open",
        )
        ctx.expect_contact(
            glovebox_door,
            dashboard_face,
            elem_a=right_hinge_barrel,
            elem_b=right_hinge_pin,
            name="right_hinge_contact_persists_open",
        )
        ctx.check(
            "door_swings_forward_when_opened",
            open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.090,
            f"Expected opened door to project forward; rest max y={rest_door_aabb[1][1]:.4f}, open max y={open_door_aabb[1][1]:.4f}.",
        )
        ctx.check(
            "door_drops_down_when_opened",
            open_door_aabb[1][2] < rest_door_aabb[1][2] - 0.020,
            f"Expected opened door top edge to drop; rest max z={rest_door_aabb[1][2]:.4f}, open max z={open_door_aabb[1][2]:.4f}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
