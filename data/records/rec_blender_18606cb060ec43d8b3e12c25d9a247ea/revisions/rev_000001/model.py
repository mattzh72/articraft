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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, z0),
            (outer_radius, z1),
        ],
        [
            (inner_radius, z0 + 0.0008),
            (inner_radius, z1 - 0.0008),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_smoothie_blender")

    body_charcoal = model.material("body_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.84, 0.93, 1.0, 0.34))
    lid_black = model.material("lid_black", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.79, 0.82, 1.0))

    base = model.part("base")
    base_shell = section_loft(
        [
            _xy_section(0.190, 0.170, 0.028, 0.000),
            _xy_section(0.178, 0.158, 0.026, 0.090),
            _xy_section(0.145, 0.132, 0.022, 0.152),
        ]
    )
    base.visual(_save_mesh("blender_base_shell", base_shell), material=body_charcoal, name="base_shell")
    base.visual(
        Box((0.126, 0.126, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=satin_steel,
        name="top_deck",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        material=dark_rubber,
        name="drive_coupling",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=dark_rubber,
        name="bayonet_socket_ring",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        base.visual(
            Box((0.014, 0.006, 0.004)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.034, math.sin(angle) * 0.034, 0.164),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_rubber,
            name=f"bayonet_guide_{index}",
        )
    base.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.089, 0.0, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="control_dial",
    )
    base.visual(
        Box((0.006, 0.014, 0.004)),
        origin=Origin(xyz=(0.098, 0.0, 0.075)),
        material=satin_steel,
        name="dial_marker",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.190, 0.170, 0.162)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
    )

    jar = model.part("jar")
    jar_shell = LatheGeometry.from_shell_profiles(
        [
            (0.022, 0.008),
            (0.060, 0.024),
            (0.072, 0.095),
            (0.070, 0.175),
            (0.054, 0.216),
            (0.040, 0.242),
            (0.039, 0.252),
        ],
        [
            (0.016, 0.012),
            (0.052, 0.028),
            (0.063, 0.097),
            (0.061, 0.173),
            (0.046, 0.214),
            (0.033, 0.238),
            (0.032, 0.247),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    jar.visual(_save_mesh("blender_jar_shell", jar_shell), material=glass_clear, name="jar_shell")
    jar.visual(
        _save_mesh(
            "blender_jar_collar",
            _ring_shell(outer_radius=0.046, inner_radius=0.018, z0=0.000, z1=0.026),
        ),
        material=lid_black,
        name="jar_collar",
    )
    jar.visual(
        _save_mesh(
            "blender_lid_seat",
            _ring_shell(outer_radius=0.041, inner_radius=0.032, z0=0.236, z1=0.252),
        ),
        material=lid_black,
        name="lid_seat",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0) + 0.35
        jar.visual(
            Box((0.014, 0.008, 0.006)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.043, math.sin(angle) * 0.043, 0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=lid_black,
            name=f"bayonet_lug_{index}",
        )
    jar.visual(
        Box((0.018, 0.028, 0.007)),
        origin=Origin(xyz=(0.040, 0.0, 0.249)),
        material=glass_clear,
        name="pour_spout",
    )
    jar.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.038, -0.014, 0.256), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="hinge_knuckle_left",
    )
    jar.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.038, 0.014, 0.256), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="hinge_knuckle_right",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.150, 0.150, 0.252)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
    )

    pour_lid = model.part("pour_lid")
    pour_lid.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.036, 0.0, 0.003)),
        material=lid_black,
        name="lid_panel",
    )
    pour_lid.visual(
        Box((0.016, 0.022, 0.006)),
        origin=Origin(xyz=(0.069, 0.0, 0.003)),
        material=lid_black,
        name="lid_pull_tab",
    )
    pour_lid.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="lid_hinge_barrel",
    )
    pour_lid.visual(
        Box((0.016, 0.024, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=lid_black,
        name="lid_hinge_bridge",
    )
    pour_lid.inertial = Inertial.from_geometry(
        Box((0.085, 0.075, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.042, 0.0, 0.004)),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_rubber,
        name="blade_hub",
    )
    blade.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_rubber,
        name="blade_shaft",
    )
    blade.visual(
        Box((0.060, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, math.radians(14.0), 0.0)),
        material=blade_steel,
        name="blade_bar_x",
    )
    blade.visual(
        Box((0.060, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.radians(-14.0), 0.0, math.pi / 2.0)),
        material=blade_steel,
        name="blade_bar_y",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.020),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_jar_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "jar_to_pour_lid",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=pour_lid,
        origin=Origin(xyz=(-0.038, 0.0, 0.252)),
        # Closed lid extends along local +X from the rear hinge line.
        # -Y makes positive q flip the pour lid upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
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

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    pour_lid = object_model.get_part("pour_lid")
    blade = object_model.get_part("blade")
    jar_lock = object_model.get_articulation("base_to_jar_lock")
    lid_hinge = object_model.get_articulation("jar_to_pour_lid")
    blade_spin = object_model.get_articulation("jar_to_blade")

    with ctx.pose({jar_lock: 0.0}):
        ctx.expect_gap(
            jar,
            base,
            axis="z",
            positive_elem="jar_collar",
            negative_elem="top_deck",
            max_gap=0.0015,
            max_penetration=0.0,
            name="jar collar seats on the deck at rest",
        )
        ctx.expect_overlap(
            jar,
            base,
            axes="xy",
            elem_a="jar_collar",
            elem_b="top_deck",
            min_overlap=0.070,
            name="jar collar stays centered on the base deck",
        )
        ctx.expect_gap(
            pour_lid,
            jar,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="lid_seat",
            max_gap=0.0015,
            max_penetration=0.0,
            name="pour lid closes onto the lid seat",
        )
        ctx.expect_overlap(
            pour_lid,
            jar,
            axes="xy",
            elem_a="lid_panel",
            elem_b="lid_seat",
            min_overlap=0.040,
            name="pour lid covers the jar opening",
        )

    jar_twist_upper = jar_lock.motion_limits.upper if jar_lock.motion_limits is not None else 0.0
    with ctx.pose({jar_lock: jar_twist_upper}):
        ctx.expect_gap(
            jar,
            base,
            axis="z",
            positive_elem="jar_collar",
            negative_elem="top_deck",
            max_gap=0.0015,
            max_penetration=0.0,
            name="jar remains seated while twisted into lock",
        )
        ctx.expect_overlap(
            jar,
            base,
            axes="xy",
            elem_a="jar_collar",
            elem_b="top_deck",
            min_overlap=0.070,
            name="jar stays on-axis during bayonet twist",
        )

    closed_lid_aabb = None
    opened_lid_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_world_aabb(pour_lid)
    lid_open_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else math.radians(90.0)
    with ctx.pose({lid_hinge: lid_open_upper}):
        opened_lid_aabb = ctx.part_world_aabb(pour_lid)
    ctx.check(
        "pour lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.02,
        details=f"closed={closed_lid_aabb}, open={opened_lid_aabb}",
    )

    ctx.check(
        "jar articulation is a short vertical bayonet twist",
        jar_lock.articulation_type == ArticulationType.REVOLUTE
        and abs(jar_lock.axis[2] - 1.0) < 1e-6
        and jar_lock.motion_limits is not None
        and jar_lock.motion_limits.lower == 0.0
        and jar_lock.motion_limits.upper is not None
        and 0.35 < jar_lock.motion_limits.upper < 0.70,
        details=f"type={jar_lock.articulation_type}, axis={jar_lock.axis}, limits={jar_lock.motion_limits}",
    )
    ctx.check(
        "pour lid uses a rear-edge revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "blade uses continuous vertical spin",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        margin=0.002,
        name="blade stays within the jar footprint",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
