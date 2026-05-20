from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campanile_bell_tower", assets=ASSETS)

    stone = model.material("stone", rgba=(0.72, 0.70, 0.66, 1.0))
    stone_trim = model.material("stone_trim", rgba=(0.80, 0.78, 0.73, 1.0))
    bronze = model.material("bronze", rgba=(0.56, 0.35, 0.16, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.20, 0.20, 0.22, 1.0))
    rope = model.material("rope", rgba=(0.58, 0.47, 0.28, 1.0))

    tower_core = model.part("tower_core")
    facade = model.part("facade")
    bell_assembly = model.part("bell_assembly")

    tower_core.inertial = Inertial.from_geometry(
        Box((0.94, 0.84, 2.72)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
    )
    facade.inertial = Inertial.from_geometry(
        Box((0.86, 0.16, 2.46)),
        mass=220.0,
        origin=Origin(xyz=(0.195, -0.01, 0.30)),
    )
    bell_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.36),
        mass=85.0,
        origin=Origin(xyz=(0.31, 0.0, -0.22)),
    )

    tower_core.visual(
        Box((0.94, 0.84, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=stone_trim,
        name="plinth",
    )
    tower_core.visual(
        Box((0.10, 0.76, 2.48)),
        origin=Origin(xyz=(-0.38, 0.0, 1.40)),
        material=stone,
        name="left_wall",
    )
    tower_core.visual(
        Box((0.10, 0.76, 2.48)),
        origin=Origin(xyz=(0.38, 0.0, 1.40)),
        material=stone,
        name="right_wall",
    )
    tower_core.visual(
        Box((0.66, 0.08, 2.48)),
        origin=Origin(xyz=(0.0, -0.36, 1.40)),
        material=stone,
        name="rear_wall",
    )
    tower_core.visual(
        Box((0.66, 0.12, 1.28)),
        origin=Origin(xyz=(0.0, 0.26, 0.80)),
        material=stone,
        name="front_backing",
    )
    tower_core.visual(
        Box((0.14, 0.04, 1.20)),
        origin=Origin(xyz=(0.0, 0.25, 0.80)),
        material=stone_trim,
        name="rope_channel_back",
    )
    tower_core.visual(
        Cylinder(radius=0.012, length=1.44),
        origin=Origin(xyz=(0.0, 0.272, 0.92)),
        material=rope,
        name="bell_rope",
    )
    tower_core.visual(
        Box((0.66, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=stone_trim,
        name="belfry_floor",
    )
    tower_core.visual(
        Box((0.90, 0.80, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 2.615)),
        material=stone_trim,
        name="cornice",
    )
    tower_core.visual(
        Box((0.94, 0.84, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.68)),
        material=stone_trim,
        name="cap_slab",
    )
    tower_core.visual(
        Box((0.03, 0.14, 0.12)),
        origin=Origin(xyz=(-0.325, 0.01, 2.08)),
        material=stone_trim,
        name="left_bracket",
    )
    tower_core.visual(
        Box((0.03, 0.14, 0.12)),
        origin=Origin(xyz=(0.325, 0.01, 2.08)),
        material=stone_trim,
        name="right_bracket",
    )

    facade.visual(
        Box((0.27, 0.10, 1.54)),
        origin=Origin(xyz=(0.0, 0.05, 0.0)),
        material=stone_trim,
        name="left_front_pier",
    )
    facade.visual(
        Box((0.27, 0.10, 1.54)),
        origin=Origin(xyz=(0.39, 0.05, 0.0)),
        material=stone_trim,
        name="right_front_pier",
    )
    facade.visual(
        Box((0.025, 0.10, 1.46)),
        origin=Origin(xyz=(0.12, 0.05, 0.0)),
        material=stone_trim,
        name="left_channel_jamb",
    )
    facade.visual(
        Box((0.025, 0.10, 1.46)),
        origin=Origin(xyz=(0.27, 0.05, 0.0)),
        material=stone_trim,
        name="right_channel_jamb",
    )
    facade.visual(
        _save_mesh("campanile_belfry_frame.obj", _build_belfry_frame()),
        origin=Origin(xyz=(0.195, 0.07, 0.67)),
        material=stone_trim,
        name="belfry_frame",
    )
    facade.visual(
        Box((0.08, 0.10, 0.14)),
        origin=Origin(xyz=(0.195, 0.05, 1.34)),
        material=stone_trim,
        name="keystone",
    )

    bell_assembly.visual(
        _save_mesh("campanile_yoke.obj", _build_yoke()),
        origin=Origin(xyz=(0.31, 0.0, -0.03)),
        material=dark_iron,
        name="yoke",
    )
    bell_assembly.visual(
        Cylinder(radius=0.033, length=0.06),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="left_sleeve",
    )
    bell_assembly.visual(
        Cylinder(radius=0.033, length=0.06),
        origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="right_sleeve",
    )
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.62),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="pivot_pin",
    )
    bell_assembly.visual(
        Box((0.16, 0.07, 0.04)),
        origin=Origin(xyz=(0.31, 0.0, -0.05)),
        material=dark_iron,
        name="bell_crown_block",
    )
    bell_assembly.visual(
        Box((0.045, 0.06, 0.10)),
        origin=Origin(xyz=(0.24, 0.0, -0.10)),
        material=dark_iron,
        name="left_hanger",
    )
    bell_assembly.visual(
        Box((0.045, 0.06, 0.10)),
        origin=Origin(xyz=(0.38, 0.0, -0.10)),
        material=dark_iron,
        name="right_hanger",
    )
    bell_assembly.visual(
        _save_mesh("campanile_bell_shell.obj", _build_bell()),
        origin=Origin(xyz=(0.31, 0.0, -0.18)),
        material=bronze,
        name="bell_shell",
    )
    bell_assembly.visual(
        Cylinder(radius=0.010, length=0.18),
        origin=Origin(xyz=(0.31, 0.0, -0.25)),
        material=dark_iron,
        name="clapper_rod",
    )
    bell_assembly.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.31, 0.0, -0.36)),
        material=dark_iron,
        name="clapper_head",
    )

    model.articulation(
        "tower_to_facade",
        ArticulationType.FIXED,
        parent=tower_core,
        child=facade,
        origin=Origin(xyz=(-0.195, 0.32, 0.93)),
    )
    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower_core,
        child=bell_assembly,
        origin=Origin(xyz=(-0.31, 0.0, 2.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=-0.30, upper=0.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower_core = object_model.get_part("tower_core")
    facade = object_model.get_part("facade")
    bell_assembly = object_model.get_part("bell_assembly")
    bell_pivot = object_model.get_articulation("tower_to_bell")

    left_wall = tower_core.get_visual("left_wall")
    rear_wall = tower_core.get_visual("rear_wall")
    front_backing = tower_core.get_visual("front_backing")
    rope_channel_back = tower_core.get_visual("rope_channel_back")
    left_bracket = tower_core.get_visual("left_bracket")
    right_bracket = tower_core.get_visual("right_bracket")
    belfry_floor = tower_core.get_visual("belfry_floor")

    left_front_pier = facade.get_visual("left_front_pier")
    belfry_frame = facade.get_visual("belfry_frame")

    bell_shell = bell_assembly.get_visual("bell_shell")
    left_sleeve = bell_assembly.get_visual("left_sleeve")
    right_sleeve = bell_assembly.get_visual("right_sleeve")
    pivot_pin = bell_assembly.get_visual("pivot_pin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    ctx.expect_contact(
        facade,
        tower_core,
        elem_a=left_front_pier,
        elem_b=front_backing,
        name="facade piers are seated on the tower face",
    )
    ctx.expect_contact(
        facade,
        tower_core,
        elem_a=belfry_frame,
        elem_b=left_wall,
        name="belfry frame keys into the tower opening surround",
    )
    ctx.expect_gap(
        facade,
        tower_core,
        axis="y",
        positive_elem=left_front_pier,
        negative_elem=rope_channel_back,
        min_gap=0.04,
        name="rope guide channel is visibly recessed below the front face",
    )
    ctx.expect_overlap(
        facade,
        bell_assembly,
        axes="xz",
        elem_a=belfry_frame,
        elem_b=bell_shell,
        min_overlap=0.14,
        name="bell reads through the arched belfry opening",
    )
    ctx.expect_gap(
        facade,
        bell_assembly,
        axis="y",
        positive_elem=belfry_frame,
        negative_elem=bell_shell,
        min_gap=0.05,
        max_gap=0.20,
        name="bell hangs just behind the front opening instead of floating deep inside",
    )
    ctx.expect_gap(
        bell_assembly,
        tower_core,
        axis="z",
        positive_elem=bell_shell,
        negative_elem=belfry_floor,
        min_gap=0.02,
        max_gap=0.10,
        name="bell mouth clears the belfry floor slab",
    )
    ctx.expect_overlap(
        tower_core,
        bell_assembly,
        axes="yz",
        elem_a=left_bracket,
        elem_b=left_sleeve,
        min_overlap=0.012,
        name="left yoke sleeve sits in the side-wall bracket zone",
    )
    ctx.expect_overlap(
        tower_core,
        bell_assembly,
        axes="yz",
        elem_a=right_bracket,
        elem_b=right_sleeve,
        min_overlap=0.012,
        name="right yoke sleeve sits in the side-wall bracket zone",
    )
    ctx.expect_gap(
        bell_assembly,
        tower_core,
        axis="x",
        positive_elem=left_sleeve,
        negative_elem=left_bracket,
        min_gap=0.0,
        max_gap=0.05,
        name="left sleeve seats just inboard of the left bracket cheek",
    )
    ctx.expect_gap(
        tower_core,
        bell_assembly,
        axis="x",
        positive_elem=right_bracket,
        negative_elem=right_sleeve,
        min_gap=0.0,
        max_gap=0.05,
        name="right sleeve seats just inboard of the right bracket cheek",
    )
    ctx.expect_contact(
        bell_assembly,
        tower_core,
        elem_a=pivot_pin,
        elem_b=left_bracket,
        name="through-bolt touches the left bracket cheek",
    )
    ctx.expect_contact(
        bell_assembly,
        tower_core,
        elem_a=pivot_pin,
        elem_b=right_bracket,
        name="through-bolt touches the right bracket cheek",
    )
    ctx.expect_within(
        bell_assembly,
        tower_core,
        axes="xy",
        inner_elem=bell_shell,
        name="bell shell stays within the tower bay footprint",
    )
    with ctx.pose({bell_pivot: 0.25}):
        ctx.expect_gap(
            facade,
            bell_assembly,
            axis="y",
            positive_elem=belfry_frame,
            negative_elem=bell_shell,
            min_gap=0.005,
            max_gap=0.24,
            name="forward swing keeps the bell behind the opening surround",
        )
        ctx.expect_within(
            bell_assembly,
            tower_core,
            axes="xy",
            inner_elem=bell_shell,
            name="forward swing stays within the tower bay",
        )
    with ctx.pose({bell_pivot: -0.25}):
        ctx.expect_gap(
            bell_assembly,
            tower_core,
            axis="y",
            positive_elem=bell_shell,
            negative_elem=rear_wall,
            min_gap=0.005,
            max_gap=0.22,
            name="rearward swing clears the back wall",
        )
        ctx.expect_within(
            bell_assembly,
            tower_core,
            axes="xy",
            inner_elem=bell_shell,
            name="rearward swing stays within the tower bay",
        )
    return ctx.report()


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _arch_profile(width: float, straight_height: float, base_z: float = 0.0, segments: int = 18):
    radius = width * 0.5
    profile = [(-radius, base_z), (radius, base_z), (radius, base_z + straight_height)]
    for index in range(segments + 1):
        angle = index * pi / segments
        profile.append((cos(angle) * radius, base_z + straight_height + sin(angle) * radius))
    profile.append((-radius, base_z + straight_height))
    return profile


def _build_belfry_frame():
    return ExtrudeWithHolesGeometry(
        [(-0.33, 0.0), (0.33, 0.0), (0.33, 0.84), (-0.33, 0.84)],
        [_arch_profile(0.44, 0.42, base_z=0.02, segments=16)],
        height=0.10,
        center=True,
    ).rotate_x(pi / 2.0)


def _build_yoke():
    outer = [
        (-0.26, 0.05),
        (-0.24, -0.01),
        (-0.21, -0.10),
        (-0.17, -0.17),
        (-0.12, -0.20),
        (-0.07, -0.18),
        (-0.05, -0.11),
        (0.05, -0.11),
        (0.07, -0.18),
        (0.12, -0.20),
        (0.17, -0.17),
        (0.21, -0.10),
        (0.24, -0.01),
        (0.26, 0.05),
        (0.20, 0.11),
        (0.10, 0.14),
        (-0.10, 0.14),
        (-0.20, 0.11),
    ]
    relief = _arch_profile(0.18, 0.08, base_z=-0.18, segments=12)
    return ExtrudeWithHolesGeometry(outer, [relief], height=0.08, center=True).rotate_x(pi / 2.0)


def _build_bell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.025, 0.12),
            (0.055, 0.10),
            (0.090, 0.04),
            (0.126, -0.05),
            (0.160, -0.16),
            (0.184, -0.28),
        ],
        [
            (0.0, 0.09),
            (0.030, 0.08),
            (0.068, 0.02),
            (0.103, -0.06),
            (0.132, -0.16),
            (0.162, -0.24),
        ],
        segments=56,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


# >>> USER_CODE_END

object_model = build_object_model()
