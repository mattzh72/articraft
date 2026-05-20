from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_loop(
    cx: float,
    cz: float,
    radius: float,
    *,
    start: float = 0.0,
    end: float = math.tau,
    samples: int = 32,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(start + (end - start) * i / samples),
            cz + radius * math.sin(start + (end - start) * i / samples),
        )
        for i in range(samples + 1)
    ]


def _annulus_y_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    mesh = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * length),
            (outer_radius, 0.5 * length),
        ],
        [
            (inner_radius, -0.5 * length),
            (inner_radius, 0.5 * length),
        ],
        segments=segments,
    )
    return mesh.rotate_x(-math.pi / 2.0)


def _blade_pack_mesh(
    *,
    blade_count: int,
    radius_center: float,
    blade_length: float,
    blade_depth: float,
    blade_thickness: float,
    sweep: float,
) -> MeshGeometry:
    pack = MeshGeometry()
    for index in range(blade_count):
        angle = index * math.tau / blade_count
        blade = BoxGeometry((blade_length, blade_depth, blade_thickness))
        blade.rotate_y(angle + sweep)
        blade.translate(
            radius_center * math.cos(angle),
            0.0,
            radius_center * math.sin(angle),
        )
        pack.merge(blade)
    return pack


def _circular_wall_mesh(
    *,
    radius: float,
    depth: float,
    thickness: float,
    segments: int,
) -> MeshGeometry:
    wall = MeshGeometry()
    segment_len = math.tau * radius / segments * 1.08
    for index in range(segments):
        angle = index * math.tau / segments
        piece = BoxGeometry((segment_len, depth, thickness))
        piece.rotate_y(angle + math.pi / 2.0)
        piece.translate(radius * math.cos(angle), 0.0, radius * math.sin(angle))
        wall.merge(piece)
    return wall


def _radial_spokes_mesh(
    *,
    count: int,
    center_radius: float,
    length: float,
    depth: float,
    thickness: float,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    spokes = MeshGeometry()
    for index in range(count):
        angle = angle_offset + index * math.tau / count
        spoke = BoxGeometry((length, depth, thickness))
        spoke.rotate_y(angle)
        spoke.translate(
            center_radius * math.cos(angle),
            0.0,
            center_radius * math.sin(angle),
        )
        spokes.merge(spoke)
    return spokes


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_blower_gpu")

    pcb_green = model.material("pcb_green", rgba=(0.12, 0.34, 0.19, 1.0))
    solder_mask_dark = model.material("solder_mask_dark", rgba=(0.08, 0.12, 0.09, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.12, 0.13, 0.14, 1.0))
    impeller_black = model.material("impeller_black", rgba=(0.08, 0.09, 0.10, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.83, 0.67, 0.28, 1.0))

    board_length = 0.267
    board_height = 0.111
    board_thickness = 0.0024
    board_front_y = 0.5 * board_thickness

    shroud_outer_y = 0.036
    shroud_plate_thickness = 0.0025
    shroud_wall_depth = shroud_outer_y - board_front_y - shroud_plate_thickness
    shroud_wall_center_y = board_front_y + 0.5 * shroud_wall_depth
    shroud_plate_center_y = shroud_outer_y - 0.5 * shroud_plate_thickness

    blower_center_x = 0.088
    blower_center_z = 0.055
    blower_outer_radius = 0.045
    blower_intake_radius = 0.033
    shroud_rear_x = -0.121
    shroud_bottom_z = blower_center_z - blower_outer_radius
    shroud_top_z = blower_center_z + blower_outer_radius

    pcb = model.part("pcb")
    pcb.visual(
        Box((board_length, board_thickness, board_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * board_height)),
        material=pcb_green,
        name="board",
    )
    pcb.visual(
        Box((0.072, 0.0012, 0.012)),
        origin=Origin(xyz=(0.008, -0.0018, 0.006)),
        material=connector_gold,
        name="edge_connector",
    )
    pcb.visual(
        Box((0.040, 0.004, 0.014)),
        origin=Origin(xyz=(-0.074, -0.0026, 0.018)),
        material=solder_mask_dark,
        name="rear_components",
    )
    pcb.visual(
        Box((0.052, 0.003, 0.020)),
        origin=Origin(xyz=(0.020, -0.0021, 0.084)),
        material=solder_mask_dark,
        name="vrm_pack",
    )
    pcb.inertial = Inertial.from_geometry(
        Box((board_length, board_thickness, board_height)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * board_height)),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.002, 0.003, 0.114)),
        origin=Origin(xyz=(-0.1345, 0.0015, 0.057)),
        material=bracket_steel,
        name="vent_frame_inner",
    )
    bracket.visual(
        Box((0.002, 0.003, 0.114)),
        origin=Origin(xyz=(-0.1345, 0.0165, 0.057)),
        material=bracket_steel,
        name="vent_frame_outer",
    )
    bracket.visual(
        Box((0.002, 0.018, 0.003)),
        origin=Origin(xyz=(-0.1345, 0.009, 0.1125)),
        material=bracket_steel,
        name="top_bar",
    )
    bracket.visual(
        Box((0.002, 0.018, 0.003)),
        origin=Origin(xyz=(-0.1345, 0.009, 0.0015)),
        material=bracket_steel,
        name="bottom_bar",
    )
    bracket.visual(
        Box((0.002, 0.018, 0.014)),
        origin=Origin(xyz=(-0.1345, 0.009, 0.099)),
        material=bracket_steel,
        name="retention_tab",
    )
    for index, z_center in enumerate((0.018, 0.031, 0.044, 0.057, 0.070, 0.083)):
        bracket.visual(
            Box((0.002, 0.016, 0.003)),
            origin=Origin(xyz=(-0.1345, 0.009, z_center)),
            material=bracket_steel,
            name=f"slot_bar_{index:02d}",
        )
    bracket.inertial = Inertial.from_geometry(
        Box((0.002, 0.018, 0.120)),
        mass=0.08,
        origin=Origin(xyz=(-0.1345, 0.009, 0.060)),
    )

    heatsink = model.part("heatsink")
    heatsink.visual(
        Box((0.154, 0.004, 0.074)),
        origin=Origin(xyz=(-0.031, board_front_y + 0.002, 0.055)),
        material=aluminum,
        name="baseplate",
    )
    fin_positions = [(-0.103 + index * 0.0066) for index in range(23)]
    for index, x_center in enumerate(fin_positions):
        heatsink.visual(
            Box((0.0016, 0.025, 0.070)),
            origin=Origin(xyz=(x_center, 0.0177, 0.055)),
            material=aluminum,
            name=f"fin_{index:02d}",
        )
    heatsink.inertial = Inertial.from_geometry(
        Box((0.154, 0.029, 0.074)),
        mass=0.62,
        origin=Origin(xyz=(-0.031, 0.0157, 0.055)),
    )

    shroud = model.part("shroud")
    shroud_outer_profile = [(shroud_rear_x, -shroud_bottom_z)]
    shroud_outer_profile.extend(
        _circle_loop(
            blower_center_x,
            -blower_center_z,
            blower_outer_radius,
            start=math.pi / 2.0,
            end=-math.pi / 2.0,
            samples=26,
        )
    )
    shroud_outer_profile.append((shroud_rear_x, -shroud_top_z))
    shroud_top_panel_geom = ExtrudeWithHolesGeometry(
        shroud_outer_profile,
        [_circle_loop(blower_center_x, -blower_center_z, blower_intake_radius, samples=40)],
        shroud_plate_thickness,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    shroud.visual(
        mesh_from_geometry(shroud_top_panel_geom, "gpu_shroud_top_panel"),
        origin=Origin(xyz=(0.0, shroud_plate_center_y, 0.0)),
        material=shroud_black,
        name="outer_shroud_panel",
    )
    shroud.visual(
        Box((0.209, shroud_wall_depth, 0.003)),
        origin=Origin(xyz=(-0.0165, shroud_wall_center_y, 0.0985)),
        material=shroud_black,
        name="tunnel_top_wall",
    )
    shroud.visual(
        Box((0.209, shroud_wall_depth, 0.003)),
        origin=Origin(xyz=(-0.0165, shroud_wall_center_y, 0.0115)),
        material=shroud_black,
        name="tunnel_bottom_wall",
    )
    shroud.visual(
        mesh_from_geometry(
            _circular_wall_mesh(
                radius=blower_outer_radius - 0.0016,
                depth=shroud_wall_depth,
                thickness=0.003,
                segments=16,
            ),
            "gpu_blower_housing_wall",
        ),
        origin=Origin(xyz=(blower_center_x, shroud_wall_center_y, blower_center_z)),
        material=shroud_black,
        name="blower_housing_wall",
    )
    shroud.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(blower_center_x, 0.0205, blower_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shroud_black,
        name="center_spindle",
    )
    shroud.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(
            xyz=(blower_center_x, 0.0155, blower_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shroud_black,
        name="lower_retainer",
    )
    shroud.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(
            xyz=(blower_center_x, 0.0285, blower_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shroud_black,
        name="upper_retainer",
    )
    shroud.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(blower_center_x, 0.0315, blower_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shroud_black,
        name="hub_support_boss",
    )
    shroud.visual(
        mesh_from_geometry(
            _radial_spokes_mesh(
                count=3,
                center_radius=0.0215,
                length=0.026,
                depth=0.0045,
                thickness=0.004,
                angle_offset=0.2,
            ),
            "gpu_blower_spider_spokes",
        ),
        origin=Origin(xyz=(blower_center_x, 0.032, blower_center_z)),
        material=shroud_black,
        name="blower_spider_spokes",
    )
    shroud.inertial = Inertial.from_geometry(
        Box((0.218, 0.036, 0.094)),
        mass=0.40,
        origin=Origin(xyz=(-0.012, 0.018, 0.055)),
    )

    impeller = model.part("impeller")
    impeller.visual(
        mesh_from_geometry(_annulus_y_mesh(0.0305, 0.0268, 0.008), "gpu_impeller_outer_ring"),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=impeller_black,
        name="cage_ring",
    )
    impeller.visual(
        mesh_from_geometry(_annulus_y_mesh(0.0103, 0.0075, 0.011), "gpu_impeller_hub_sleeve"),
        origin=Origin(),
        material=impeller_black,
        name="hub_sleeve",
    )
    impeller.visual(
        mesh_from_geometry(_annulus_y_mesh(0.0300, 0.0098, 0.0018), "gpu_impeller_backplate"),
        origin=Origin(xyz=(0.0, -0.0046, 0.0)),
        material=impeller_black,
        name="backplate",
    )
    impeller.visual(
        mesh_from_geometry(
            _blade_pack_mesh(
                blade_count=14,
                radius_center=0.019,
                blade_length=0.016,
                blade_depth=0.008,
                blade_thickness=0.0022,
                sweep=0.38,
            ),
            "gpu_impeller_blades",
        ),
        origin=Origin(),
        material=impeller_black,
        name="blade_pack",
    )
    impeller.visual(
        Cylinder(radius=0.012, length=0.0015),
        origin=Origin(
            xyz=(0.0, 0.0046, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=impeller_black,
        name="hub_cap",
    )
    impeller.inertial = Inertial.from_geometry(
        Box((0.064, 0.013, 0.064)),
        mass=0.05,
        origin=Origin(xyz=(blower_center_x, 0.022, blower_center_z)),
    )

    model.articulation(
        "pcb_to_bracket",
        ArticulationType.FIXED,
        parent=pcb,
        child=bracket,
        origin=Origin(),
    )
    model.articulation(
        "pcb_to_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=heatsink,
        origin=Origin(),
    )
    model.articulation(
        "pcb_to_shroud",
        ArticulationType.FIXED,
        parent=pcb,
        child=shroud,
        origin=Origin(),
    )
    model.articulation(
        "blower_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=impeller,
        origin=Origin(xyz=(blower_center_x, 0.022, blower_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=300.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    pcb = object_model.get_part("pcb")
    bracket = object_model.get_part("bracket")
    heatsink = object_model.get_part("heatsink")
    shroud = object_model.get_part("shroud")
    impeller = object_model.get_part("impeller")
    blower_spin = object_model.get_articulation("blower_spin")

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

    ctx.check(
        "blower articulation is continuous",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous blower spin joint, got {blower_spin.articulation_type!r}.",
    )
    ctx.check(
        "blower articulation axis",
        tuple(blower_spin.axis) == (0.0, 1.0, 0.0),
        details=f"Expected blower axis along +Y, got {blower_spin.axis!r}.",
    )

    ctx.expect_contact(bracket, pcb, name="bracket touches pcb edge")
    ctx.expect_contact(heatsink, pcb, elem_a="baseplate", elem_b="board", name="heatsink seats on pcb")
    ctx.expect_contact(shroud, pcb, elem_a="tunnel_top_wall", elem_b="board", name="shroud mounts to pcb")
    ctx.expect_within(heatsink, shroud, axes="xz", margin=0.0, name="heatsink stays inside shroud tunnel")
    ctx.expect_overlap(heatsink, bracket, axes="yz", min_overlap=0.010, name="heatsink aligns with bracket exhaust")
    ctx.expect_gap(
        impeller,
        pcb,
        axis="y",
        min_gap=0.014,
        positive_elem="blade_pack",
        negative_elem="board",
        name="impeller clears pcb",
    )
    ctx.expect_within(
        impeller,
        shroud,
        axes="xz",
        margin=0.0,
        inner_elem="blade_pack",
        outer_elem="blower_housing_wall",
        name="impeller sits within blower housing footprint",
    )
    ctx.expect_gap(
        impeller,
        shroud,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="hub_sleeve",
        negative_elem="lower_retainer",
        name="impeller rests above lower retainer",
    )
    ctx.expect_gap(
        shroud,
        impeller,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="upper_retainer",
        negative_elem="hub_sleeve",
        name="upper retainer captures impeller hub",
    )

    with ctx.pose({blower_spin: 2.2}):
        ctx.expect_within(
            impeller,
            shroud,
            axes="xz",
            margin=0.0,
            inner_elem="blade_pack",
            outer_elem="blower_housing_wall",
            name="spun impeller stays inside blower housing footprint",
        )
        ctx.expect_gap(
            impeller,
            pcb,
            axis="y",
            min_gap=0.014,
            positive_elem="blade_pack",
            negative_elem="board",
            name="spun impeller still clears pcb",
        )
        ctx.expect_gap(
            shroud,
            impeller,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="upper_retainer",
            negative_elem="hub_sleeve",
            name="spun impeller remains clipped under upper retainer",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
