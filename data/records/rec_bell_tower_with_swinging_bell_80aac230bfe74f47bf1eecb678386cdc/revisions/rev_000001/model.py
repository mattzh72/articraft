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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _pointed_arch_profile(
    *,
    width: float,
    sill: float,
    shoulder: float,
    apex: float,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    rise = apex - shoulder
    return [
        (-half_w, sill),
        (half_w, sill),
        (half_w, shoulder),
        (half_w * 0.62, shoulder + rise * 0.30),
        (half_w * 0.30, shoulder + rise * 0.78),
        (0.0, apex),
        (-half_w * 0.30, shoulder + rise * 0.78),
        (-half_w * 0.62, shoulder + rise * 0.30),
        (-half_w, shoulder),
    ]


def _octagon_loop(radius: float, z: float, *, phase: float = math.pi / 8.0) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(phase + (index * math.pi / 4.0)),
            radius * math.sin(phase + (index * math.pi / 4.0)),
            z,
        )
        for index in range(8)
    ]


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        0.5 * (mins[0] + maxs[0]),
        0.5 * (mins[1] + maxs[1]),
        0.5 * (mins[2] + maxs[2]),
    )


def _add_bell_assembly(
    model: ArticulatedObject,
    *,
    side_name: str,
    x_pos: float,
    axle_z: float,
    bell_shell_mesh,
    timber,
    bronze,
    dark_iron,
):
    bell = model.part(f"{side_name}_bell")
    bell.visual(
        Cylinder(radius=0.04, length=0.74),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    bell.visual(
        Box((0.58, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=timber,
        name="yoke",
    )
    bell.visual(
        Box((0.38, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=timber,
        name="yoke_cap",
    )
    bell.visual(
        Box((0.34, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=timber,
        name="crown_block",
    )
    bell.visual(
        Box((0.07, 0.04, 0.16)),
        origin=Origin(xyz=(-0.12, 0.0, -0.10)),
        material=dark_iron,
        name="hanger_neg_x",
    )
    bell.visual(
        Box((0.07, 0.04, 0.16)),
        origin=Origin(xyz=(0.12, 0.0, -0.10)),
        material=dark_iron,
        name="hanger_pos_x",
    )
    bell.visual(
        bell_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(-0.165, 0.0, -0.30)),
        material=dark_iron,
        name="pin_lug_neg_x",
    )
    bell.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(0.165, 0.0, -0.30)),
        material=dark_iron,
        name="pin_lug_pos_x",
    )
    bell.inertial = Inertial.from_geometry(
        Box((1.35, 1.00, 1.55)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
    )

    clapper = model.part(f"{side_name}_clapper")
    clapper.visual(
        Cylinder(radius=0.014, length=0.28),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="clapper_pin",
    )
    clapper.visual(
        Cylinder(radius=0.032, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        material=dark_iron,
        name="clapper_stem",
    )
    clapper.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.69)),
        material=dark_iron,
        name="clapper_bob",
    )
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.09, length=0.80),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
    )

    bell_joint = model.articulation(
        f"{side_name}_bell_swing",
        ArticulationType.REVOLUTE,
        parent="tower",
        child=bell,
        origin=Origin(xyz=(x_pos, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=1.2,
            lower=-0.95,
            upper=0.95,
        ),
    )
    clapper_joint = model.articulation(
        f"{side_name}_clapper_pivot",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=3.0,
            lower=-0.85,
            upper=0.85,
        ),
    )

    return bell, clapper, bell_joint, clapper_joint


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_belfry_tower")

    weathered_stone = model.material("weathered_stone", rgba=(0.66, 0.66, 0.68, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.52, 0.52, 0.54, 1.0))
    oak_timber = model.material("oak_timber", rgba=(0.43, 0.29, 0.17, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.67, 0.49, 0.22, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.20, 1.0))

    plinth_h = 0.90
    shaft_h = 14.40
    belfry_h = 6.20
    roof_h = 0.50
    drum_h = 2.90
    spire_h = 9.70
    shaft_top_z = plinth_h + shaft_h
    belfry_top_z = shaft_top_z + belfry_h
    roof_top_z = belfry_top_z + roof_h
    drum_top_z = roof_top_z + drum_h
    axle_z = shaft_top_z + 3.35

    twin_arch = list(
        reversed(
            _pointed_arch_profile(
                width=1.35,
                sill=-2.08,
                shoulder=0.72,
                apex=1.92,
            )
        )
    )
    belfry_wall_geom = ExtrudeWithHolesGeometry(
        _rect_profile(6.00, 6.20),
        [
            _translate_profile(twin_arch, dx=-1.15),
            _translate_profile(twin_arch, dx=1.15),
        ],
        0.45,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    belfry_wall_mesh = _save_mesh("gothic_belfry_wall", belfry_wall_geom)

    drum_mesh = _save_mesh(
        "ribbed_drum_shell",
        section_loft(
            [
                _octagon_loop(2.18, 0.0),
                _octagon_loop(2.10, drum_h * 0.45),
                _octagon_loop(2.02, drum_h),
            ]
        ),
    )
    spire_mesh = _save_mesh(
        "octagonal_spire_shell",
        section_loft(
            [
                _octagon_loop(1.96, 0.0),
                _octagon_loop(1.48, spire_h * 0.34),
                _octagon_loop(0.94, spire_h * 0.68),
                _octagon_loop(0.34, spire_h * 0.92),
                _octagon_loop(0.04, spire_h),
            ]
        ),
    )
    bell_shell_mesh = _save_mesh(
        "cathedral_bell_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.11, 0.06),
                (0.18, 0.05),
                (0.31, -0.03),
                (0.45, -0.28),
                (0.56, -0.64),
                (0.61, -0.95),
                (0.60, -1.05),
                (0.63, -1.12),
            ],
            [
                (0.05, 0.02),
                (0.11, 0.00),
                (0.22, -0.07),
                (0.35, -0.29),
                (0.46, -0.61),
                (0.52, -0.90),
                (0.56, -1.04),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )

    tower = model.part("tower")
    tower.visual(
        Box((7.40, 7.40, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h * 0.5)),
        material=dark_stone,
        name="foundation_plinth",
    )
    tower.visual(
        Box((6.20, 6.20, shaft_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + (shaft_h * 0.5))),
        material=weathered_stone,
        name="main_shaft",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.85, 0.85, shaft_h + 0.60)),
                origin=Origin(
                    xyz=(
                        x_sign * 3.12,
                        y_sign * 3.12,
                        0.5 * (shaft_h + 0.60),
                    )
                ),
                material=dark_stone,
                name=f"buttress_{'e' if x_sign > 0 else 'w'}_{'n' if y_sign > 0 else 's'}",
            )
    tower.visual(
        Box((6.90, 6.90, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, shaft_top_z + 0.15)),
        material=dark_stone,
        name="shaft_stringcourse",
    )
    tower.visual(
        Box((5.95, 5.95, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, shaft_top_z + 0.225)),
        material=weathered_stone,
        name="belfry_floor",
    )
    tower.visual(
        belfry_wall_mesh,
        origin=Origin(xyz=(0.0, 3.175, shaft_top_z + (belfry_h * 0.5))),
        material=weathered_stone,
        name="front_belfry_wall",
    )
    tower.visual(
        belfry_wall_mesh,
        origin=Origin(
            xyz=(0.0, -3.175, shaft_top_z + (belfry_h * 0.5)),
            rpy=(0.0, 0.0, math.pi),
        ),
        material=weathered_stone,
        name="rear_belfry_wall",
    )
    tower.visual(
        belfry_wall_mesh,
        origin=Origin(
            xyz=(3.175, 0.0, shaft_top_z + (belfry_h * 0.5)),
            rpy=(0.0, 0.0, math.pi / 2.0),
        ),
        material=weathered_stone,
        name="east_belfry_wall",
    )
    tower.visual(
        belfry_wall_mesh,
        origin=Origin(
            xyz=(-3.175, 0.0, shaft_top_z + (belfry_h * 0.5)),
            rpy=(0.0, 0.0, -math.pi / 2.0),
        ),
        material=weathered_stone,
        name="west_belfry_wall",
    )
    tower.visual(
        Box((6.00, 0.34, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, axle_z + 0.30)),
        material=oak_timber,
        name="bell_frame_crossbeam",
    )
    for side_name, x_pos in (("left", -1.35), ("right", 1.35)):
        for label, x_shift in (("neg_x", -0.43), ("pos_x", 0.43)):
            tower.visual(
                Box((0.12, 0.26, 0.34)),
                origin=Origin(xyz=(x_pos + x_shift, 0.0, axle_z + 0.14)),
                material=oak_timber,
                name=f"{side_name}_bell_bearing_{label}",
            )
    tower.visual(
        Box((6.95, 6.95, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, belfry_top_z + 0.14)),
        material=dark_stone,
        name="belfry_cornice",
    )
    tower.visual(
        Box((6.25, 6.25, roof_h)),
        origin=Origin(xyz=(0.0, 0.0, belfry_top_z + (roof_h * 0.5))),
        material=weathered_stone,
        name="spire_base_slab",
    )
    tower.visual(
        drum_mesh,
        origin=Origin(xyz=(0.0, 0.0, roof_top_z)),
        material=weathered_stone,
        name="ribbed_drum",
    )
    for index in range(8):
        angle = (math.pi / 8.0) + (index * math.pi / 4.0)
        tower.visual(
            Box((0.30, 0.18, drum_h + 0.02)),
            origin=Origin(
                xyz=(
                    2.06 * math.cos(angle),
                    2.06 * math.sin(angle),
                    roof_top_z + (drum_h * 0.5),
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_stone,
            name=f"drum_rib_{index:02d}",
        )
    tower.visual(
        spire_mesh,
        origin=Origin(xyz=(0.0, 0.0, drum_top_z)),
        material=dark_stone,
        name="octagonal_spire",
    )
    tower.visual(
        Cylinder(radius=0.05, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, drum_top_z + spire_h + 0.55)),
        material=dark_iron,
        name="spire_finial",
    )
    tower.inertial = Inertial.from_geometry(
        Box((7.40, 7.40, drum_top_z + spire_h)),
        mass=180000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (drum_top_z + spire_h))),
    )

    _add_bell_assembly(
        model,
        side_name="left",
        x_pos=-1.35,
        axle_z=axle_z,
        bell_shell_mesh=bell_shell_mesh,
        timber=oak_timber,
        bronze=bell_bronze,
        dark_iron=dark_iron,
    )
    _add_bell_assembly(
        model,
        side_name="right",
        x_pos=1.35,
        axle_z=axle_z,
        bell_shell_mesh=bell_shell_mesh,
        timber=oak_timber,
        bronze=bell_bronze,
        dark_iron=dark_iron,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    left_bell = object_model.get_part("left_bell")
    right_bell = object_model.get_part("right_bell")
    left_clapper = object_model.get_part("left_clapper")
    right_clapper = object_model.get_part("right_clapper")

    left_bell_joint = object_model.get_articulation("left_bell_swing")
    right_bell_joint = object_model.get_articulation("right_bell_swing")
    left_clapper_joint = object_model.get_articulation("left_clapper_pivot")
    right_clapper_joint = object_model.get_articulation("right_clapper_pivot")

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

    ctx.check(
        "bell and clapper pivots stay on horizontal x axes",
        left_bell_joint.axis == (1.0, 0.0, 0.0)
        and right_bell_joint.axis == (1.0, 0.0, 0.0)
        and left_clapper_joint.axis == (1.0, 0.0, 0.0)
        and right_clapper_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"bell_axes=({left_bell_joint.axis}, {right_bell_joint.axis}), "
            f"clapper_axes=({left_clapper_joint.axis}, {right_clapper_joint.axis})"
        ),
    )

    ctx.expect_contact(
        left_bell,
        tower,
        elem_a="axle",
        elem_b="left_bell_bearing_neg_x",
        name="left bell axle bears on the outer timber hanger",
    )
    ctx.expect_contact(
        left_bell,
        tower,
        elem_a="axle",
        elem_b="left_bell_bearing_pos_x",
        name="left bell axle bears on the inner timber hanger",
    )
    ctx.expect_contact(
        right_bell,
        tower,
        elem_a="axle",
        elem_b="right_bell_bearing_neg_x",
        name="right bell axle bears on the inner timber hanger",
    )
    ctx.expect_contact(
        right_bell,
        tower,
        elem_a="axle",
        elem_b="right_bell_bearing_pos_x",
        name="right bell axle bears on the outer timber hanger",
    )

    ctx.expect_contact(
        left_clapper,
        left_bell,
        elem_a="clapper_pin",
        elem_b="pin_lug_neg_x",
        name="left clapper hangs from its first pivot lug",
    )
    ctx.expect_contact(
        left_clapper,
        left_bell,
        elem_a="clapper_pin",
        elem_b="pin_lug_pos_x",
        name="left clapper hangs from its second pivot lug",
    )
    ctx.expect_contact(
        right_clapper,
        right_bell,
        elem_a="clapper_pin",
        elem_b="pin_lug_neg_x",
        name="right clapper hangs from its first pivot lug",
    )
    ctx.expect_contact(
        right_clapper,
        right_bell,
        elem_a="clapper_pin",
        elem_b="pin_lug_pos_x",
        name="right clapper hangs from its second pivot lug",
    )

    ctx.expect_origin_distance(
        left_bell,
        right_bell,
        axes="x",
        min_dist=2.50,
        max_dist=2.90,
        name="paired bells occupy separate belfry bays",
    )
    ctx.expect_gap(
        left_bell,
        tower,
        axis="z",
        positive_elem="bell_shell",
        negative_elem="belfry_floor",
        min_gap=1.30,
        max_gap=1.90,
        name="left bell mouth hangs high above the belfry floor",
    )
    ctx.expect_gap(
        right_bell,
        tower,
        axis="z",
        positive_elem="bell_shell",
        negative_elem="belfry_floor",
        min_gap=1.30,
        max_gap=1.90,
        name="right bell mouth hangs high above the belfry floor",
    )

    left_rest_center = _aabb_center(ctx.part_element_world_aabb(left_bell, elem="bell_shell"))
    with ctx.pose({left_bell_joint: 0.75}):
        left_swung_center = _aabb_center(ctx.part_element_world_aabb(left_bell, elem="bell_shell"))
        ctx.expect_gap(
            tower,
            left_bell,
            axis="y",
            positive_elem="front_belfry_wall",
            negative_elem="bell_shell",
            min_gap=0.15,
            name="left bell keeps wall clearance near a forward ringing pose",
        )
    ctx.check(
        "positive left bell swing drives the bell toward the front arch",
        left_rest_center is not None
        and left_swung_center is not None
        and left_swung_center[1] > left_rest_center[1] + 0.30,
        details=f"rest={left_rest_center}, swung={left_swung_center}",
    )

    clapper_rest_center = _aabb_center(ctx.part_element_world_aabb(left_clapper, elem="clapper_bob"))
    with ctx.pose({left_clapper_joint: 0.45}):
        clapper_swung_center = _aabb_center(ctx.part_element_world_aabb(left_clapper, elem="clapper_bob"))
    ctx.check(
        "left clapper swings independently on its secondary pin",
        clapper_rest_center is not None
        and clapper_swung_center is not None
        and clapper_swung_center[1] > clapper_rest_center[1] + 0.08,
        details=f"rest={clapper_rest_center}, swung={clapper_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
