from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blower_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shroud_gray = model.material("shroud_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.27, 0.29, 0.32, 1.0))
    fan_black = model.material("fan_black", rgba=(0.06, 0.06, 0.07, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.09, 0.22, 0.12, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.79, 0.67, 0.24, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.16, 0.18, 0.19, 1.0))

    card_length = 0.186
    card_width = 0.040
    card_height = 0.108
    shell_thickness = 0.0025
    top_bottom_thickness = 0.003
    half_length = card_length * 0.5
    half_width = card_width * 0.5
    half_height = card_height * 0.5

    impeller_center_x = 0.033
    impeller_center_z = 0.006
    impeller_outer_radius = 0.026
    intake_bezel_radius = 0.0325
    hatch_width = 0.028
    hatch_height = 0.024
    hatch_center_x = -0.020
    hatch_center_z = 0.014

    intake_panel_geom = ExtrudeWithHolesGeometry(
        _rect_profile(card_length - 0.002, card_height - 0.002),
        [
            _circle_profile(intake_bezel_radius - 0.0015, center=(impeller_center_x, impeller_center_z), segments=44),
            _rect_profile(hatch_width + 0.0012, hatch_height + 0.0012, center=(hatch_center_x, hatch_center_z)),
        ],
        height=shell_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    intake_panel_mesh = mesh_from_geometry(intake_panel_geom, "gpu_intake_panel")

    intake_bezel_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=intake_bezel_radius - 0.003,
            tube=0.0024,
            radial_segments=18,
            tubular_segments=56,
        ).rotate_x(math.pi / 2.0),
        "gpu_intake_bezel",
    )
    impeller_tip_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.0228,
            tube=0.0014,
            radial_segments=16,
            tubular_segments=48,
        ).rotate_x(math.pi / 2.0),
        "gpu_impeller_tip_ring",
    )

    shroud = model.part("shroud")
    shroud.visual(
        Box((card_length - 0.010, card_width - 0.004, top_bottom_thickness)),
        origin=Origin(xyz=(0.000, 0.000, half_height - top_bottom_thickness * 0.5)),
        material=shroud_black,
        name="top_cover",
    )
    shroud.visual(
        Box((card_length, card_width - 0.004, top_bottom_thickness)),
        origin=Origin(xyz=(0.000, 0.000, -half_height + top_bottom_thickness * 0.5)),
        material=shroud_black,
        name="bottom_plate",
    )
    shroud.visual(
        Box((card_length - 0.004, shell_thickness, card_height - 0.006)),
        origin=Origin(xyz=(0.000, -half_width + shell_thickness * 0.5, 0.000)),
        material=shroud_gray,
        name="rear_panel",
    )
    shroud.visual(
        intake_panel_mesh,
        origin=Origin(xyz=(0.000, half_width - shell_thickness * 0.5, 0.000)),
        material=shroud_gray,
        name="intake_panel",
    )
    shroud.visual(
        intake_bezel_mesh,
        origin=Origin(xyz=(impeller_center_x, half_width - shell_thickness * 0.5, impeller_center_z)),
        material=dark_metal,
        name="intake_bezel",
    )
    shroud.visual(
        Box((shell_thickness, card_width - 0.004, card_height - 0.006)),
        origin=Origin(xyz=(-half_length + shell_thickness * 0.5, 0.000, 0.000)),
        material=dark_metal,
        name="io_bracket_plate",
    )
    shroud.visual(
        Box((shell_thickness, card_width - 0.004, card_height - 0.006)),
        origin=Origin(xyz=(half_length - shell_thickness * 0.5, 0.000, 0.000)),
        material=shroud_gray,
        name="far_end_cap",
    )
    for vent_index, vent_z in enumerate((-0.030, -0.013, 0.004, 0.021, 0.038)):
        shroud.visual(
            Box((0.005, card_width - 0.010, 0.004)),
            origin=Origin(xyz=(-half_length + 0.0035, -0.0005, vent_z)),
            material=dark_metal,
            name=f"exhaust_fin_{vent_index}",
        )

    shroud.visual(
        Cylinder(radius=0.035, length=0.0022),
        origin=Origin(
            xyz=(impeller_center_x, -0.0091, impeller_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="blower_backplate",
    )
    shroud.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(
            xyz=(impeller_center_x, -0.0130, impeller_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="blower_spindle",
    )
    shroud.visual(
        Box((0.074, 0.020, 0.011)),
        origin=Origin(xyz=(-0.024, 0.000, impeller_center_z - 0.031)),
        material=dark_metal,
        name="exhaust_duct_floor",
    )
    shroud.visual(
        Box((0.093, 0.010, 0.016)),
        origin=Origin(xyz=(-0.044, -0.004, impeller_center_z + 0.030)),
        material=dark_metal,
        name="scroll_cutoff_wall",
    )
    shroud.visual(
        Box((0.036, 0.0023, 0.004)),
        origin=Origin(xyz=(hatch_center_x, half_width - 0.00075, hatch_center_z + hatch_height * 0.5)),
        material=shroud_gray,
        name="hatch_hinge_mount",
    )
    shroud.visual(
        Box((0.031, 0.010, 0.002)),
        origin=Origin(xyz=(hatch_center_x, half_width - 0.007, hatch_center_z)),
        material=filter_dark,
        name="dust_filter",
    )
    shroud.visual(
        Box((0.152, 0.030, 0.002)),
        origin=Origin(xyz=(-0.003, 0.000, -half_height + 0.0035)),
        material=pcb_green,
        name="pcb",
    )
    shroud.visual(
        Box((0.108, 0.008, 0.002)),
        origin=Origin(xyz=(0.008, 0.000, -half_height + 0.002)),
        material=connector_gold,
        name="pcie_edge",
    )
    shroud.inertial = Inertial.from_geometry(
        Box((card_length, card_width, card_height)),
        mass=0.85,
    )

    impeller = model.part("impeller")
    impeller.visual(
        Cylinder(radius=impeller_outer_radius, length=0.002),
        origin=Origin(xyz=(0.000, -0.0065, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_black,
        name="impeller_back_disc",
    )
    impeller.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fan_black,
        name="impeller_hub",
    )
    impeller.visual(
        Sphere(radius=0.0078),
        origin=Origin(xyz=(0.000, 0.0040, 0.000)),
        material=fan_black,
        name="impeller_nose",
    )
    impeller.visual(
        impeller_tip_ring_mesh,
        origin=Origin(xyz=(0.000, 0.0035, 0.000)),
        material=fan_black,
        name="impeller_tip_ring",
    )
    blade_count = 11
    for blade_index in range(blade_count):
        angle = 2.0 * math.pi * blade_index / blade_count
        blade_radius = 0.0165
        impeller.visual(
            Box((0.017, 0.010, 0.0024)),
            origin=Origin(
                xyz=(
                    blade_radius * math.cos(angle),
                    -0.0005,
                    blade_radius * math.sin(angle),
                ),
                rpy=(0.0, angle + math.radians(24.0), 0.0),
            ),
            material=fan_black,
            name=f"impeller_blade_{blade_index:02d}",
        )
    impeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.016),
        mass=0.08,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    hatch = model.part("service_hatch")
    hatch.visual(
        Box((hatch_width, 0.0020, hatch_height)),
        origin=Origin(xyz=(0.000, 0.0002, -hatch_height * 0.5)),
        material=shroud_gray,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.0017, length=hatch_width * 0.72),
        origin=Origin(xyz=(0.000, 0.0004, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hatch_barrel",
    )
    hatch.visual(
        Box((0.010, 0.0035, 0.003)),
        origin=Origin(xyz=(0.000, 0.0020, -hatch_height + 0.0030)),
        material=dark_metal,
        name="hatch_pull_tab",
    )
    hatch.inertial = Inertial.from_geometry(
        Box((hatch_width, 0.004, hatch_height)),
        mass=0.018,
        origin=Origin(xyz=(0.000, 0.0010, -hatch_height * 0.5)),
    )

    model.articulation(
        "shroud_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=impeller,
        origin=Origin(xyz=(impeller_center_x, 0.000, impeller_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
    )
    model.articulation(
        "shroud_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=shroud,
        child=hatch,
        origin=Origin(
            xyz=(
                hatch_center_x,
                half_width + 0.0017,
                hatch_center_z + hatch_height * 0.5,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
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
    shroud = object_model.get_part("shroud")
    impeller = object_model.get_part("impeller")
    hatch = object_model.get_part("service_hatch")
    impeller_joint = object_model.get_articulation("shroud_to_impeller")
    hatch_joint = object_model.get_articulation("shroud_to_service_hatch")

    ctx.expect_overlap(
        impeller,
        shroud,
        axes="xz",
        elem_a="impeller_tip_ring",
        elem_b="intake_bezel",
        min_overlap=0.048,
        name="impeller stays centered behind the intake bezel",
    )
    ctx.expect_within(
        impeller,
        shroud,
        axes="xz",
        margin=0.002,
        name="impeller remains within the card footprint",
    )

    impeller_rest_pos = ctx.part_world_position(impeller)
    with ctx.pose({impeller_joint: math.pi / 2.0}):
        impeller_spin_pos = ctx.part_world_position(impeller)
    ctx.check(
        "impeller spins in place",
        impeller_rest_pos is not None
        and impeller_spin_pos is not None
        and max(abs(a - b) for a, b in zip(impeller_rest_pos, impeller_spin_pos)) <= 1e-6,
        details=f"rest={impeller_rest_pos}, spun={impeller_spin_pos}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    closed_pull_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_pull_tab")
    with ctx.pose({hatch_joint: math.radians(70.0)}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
        open_pull_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_pull_tab")
    ctx.check(
        "service hatch swings outward from the intake panel",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][1] > closed_hatch_aabb[1][1] + 0.010,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )
    ctx.check(
        "hatch pull tab clears farther out when opened",
        closed_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[1][1] > closed_pull_aabb[1][1] + 0.012,
        details=f"closed={closed_pull_aabb}, open={open_pull_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
