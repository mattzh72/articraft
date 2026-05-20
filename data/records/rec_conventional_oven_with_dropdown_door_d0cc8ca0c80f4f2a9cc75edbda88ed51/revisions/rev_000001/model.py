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
    ExtrudeGeometry,
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


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _arch_profile(width: float, height: float, *, arc_segments: int = 18) -> list[tuple[float, float]]:
    half_width = width * 0.5
    crown_radius = half_width
    shoulder_height = max(0.0, height - crown_radius)

    points: list[tuple[float, float]] = [
        (half_width, 0.0),
        (half_width, shoulder_height),
    ]
    for index in range(1, arc_segments):
        theta = math.pi * index / arc_segments
        points.append(
            (
                half_width * math.cos(theta),
                shoulder_height + crown_radius * math.sin(theta),
            )
        )
    points.extend(
        [
            (-half_width, shoulder_height),
            (-half_width, 0.0),
        ]
    )
    return points


def _build_dome_shell_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.430, 0.000),
            (0.424, 0.070),
            (0.404, 0.175),
            (0.348, 0.302),
            (0.255, 0.422),
            (0.118, 0.505),
            (0.032, 0.538),
            (0.006, 0.548),
            (0.000, 0.548),
        ],
        segments=80,
    )


def _build_entry_arch_mesh() -> MeshGeometry:
    outer_profile = _arch_profile(0.580, 0.350, arc_segments=22)
    inner_profile = _arch_profile(0.400, 0.240, arc_segments=18)
    return (
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            0.160,
            cap=True,
            center=True,
            closed=True,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.430, 0.0)
    )


def _build_door_panel_mesh() -> MeshGeometry:
    return (
        ExtrudeGeometry(
            _arch_profile(0.388, 0.228, arc_segments=18),
            0.008,
            cap=True,
            center=True,
            closed=True,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.004, 0.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_fired_pizza_oven")

    stone = model.material("stone", rgba=(0.76, 0.72, 0.66, 1.0))
    refractory = model.material("refractory", rgba=(0.88, 0.84, 0.76, 1.0))
    soot_black = model.material("soot_black", rgba=(0.14, 0.12, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    wood = model.material("wood", rgba=(0.32, 0.21, 0.12, 1.0))

    base = model.part("platform_base")
    base.visual(
        Box((1.160, 0.880, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=stone,
        name="masonry_block",
    )
    base.visual(
        Box((1.220, 0.940, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=stone,
        name="counter_slab",
    )
    base.visual(
        Box((0.920, 0.860, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=refractory,
        name="firebox_top",
    )
    base.visual(
        Box((0.080, 0.860, 0.120)),
        origin=Origin(xyz=(-0.420, 0.0, 0.740)),
        material=refractory,
        name="firebox_left_wall",
    )
    base.visual(
        Box((0.080, 0.860, 0.120)),
        origin=Origin(xyz=(0.420, 0.0, 0.740)),
        material=refractory,
        name="firebox_right_wall",
    )
    base.visual(
        Box((0.760, 0.060, 0.120)),
        origin=Origin(xyz=(0.0, 0.400, 0.740)),
        material=refractory,
        name="firebox_back_wall",
    )
    base.visual(
        Box((0.540, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, -0.405, 0.740)),
        material=soot_black,
        name="drawer_shadow_header",
    )
    base.visual(
        Box((0.500, 0.560, 0.006)),
        origin=Origin(xyz=(0.0, -0.115, 0.683)),
        material=dark_steel,
        name="drawer_guide_floor",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.220, 0.940, 0.800)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
    )

    oven = model.part("oven_body")
    oven.visual(
        mesh_from_geometry(_build_dome_shell_mesh(), "dome_shell"),
        material=stone,
        name="dome_shell",
    )
    oven.visual(
        mesh_from_geometry(_build_entry_arch_mesh(), "entry_arch"),
        material=refractory,
        name="entry_arch",
    )
    oven.visual(
        Box((0.680, 0.720, 0.025)),
        origin=Origin(xyz=(0.0, 0.030, 0.0125)),
        material=refractory,
        name="hearth_stone",
    )
    oven.visual(
        Box((0.500, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, -0.370, 0.009)),
        material=refractory,
        name="landing_stone",
    )
    oven.visual(
        Box((0.330, 0.120, 0.150)),
        origin=Origin(xyz=(0.0, -0.295, 0.105)),
        material=soot_black,
        name="chamber_shadow",
    )
    oven.inertial = Inertial.from_geometry(
        Cylinder(radius=0.430, length=0.560),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(_build_door_panel_mesh(), "door_panel"),
        material=steel,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.014, 0.034)),
        origin=Origin(xyz=(-0.070, -0.015, 0.146)),
        material=dark_steel,
        name="left_handle_post",
    )
    door.visual(
        Box((0.018, 0.014, 0.034)),
        origin=Origin(xyz=(0.070, -0.015, 0.146)),
        material=dark_steel,
        name="right_handle_post",
    )
    door.visual(
        Box((0.175, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.024, 0.146)),
        material=wood,
        name="handle_grip",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.390, 0.040, 0.235)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.012, 0.118)),
    )

    drawer = model.part("ash_drawer")
    drawer.visual(
        Box((0.500, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, -0.006, -0.032)),
        material=steel,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.452, 0.520, 0.004)),
        origin=Origin(xyz=(0.0, 0.260, -0.019)),
        material=steel,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.004, 0.520, 0.036)),
        origin=Origin(xyz=(-0.224, 0.260, -0.001)),
        material=steel,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((0.004, 0.520, 0.036)),
        origin=Origin(xyz=(0.224, 0.260, -0.001)),
        material=steel,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((0.452, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, 0.518, -0.001)),
        material=steel,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((0.016, 0.016, 0.020)),
        origin=Origin(xyz=(-0.070, -0.020, -0.026)),
        material=dark_steel,
        name="drawer_handle_left_post",
    )
    drawer.visual(
        Box((0.016, 0.016, 0.020)),
        origin=Origin(xyz=(0.070, -0.020, -0.026)),
        material=dark_steel,
        name="drawer_handle_right_post",
    )
    drawer.visual(
        Box((0.160, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.032, -0.026)),
        material=wood,
        name="drawer_handle_grip",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.500, 0.520, 0.040)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.254, -0.001)),
    )

    model.articulation(
        "base_to_oven",
        ArticulationType.FIXED,
        parent=base,
        child=oven,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
    )
    model.articulation(
        "oven_to_door",
        ArticulationType.REVOLUTE,
        parent=oven,
        child=door,
        origin=Origin(xyz=(0.0, -0.510, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "oven_to_ash_drawer",
        ArticulationType.PRISMATIC,
        parent=oven,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.474, -0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=0.260,
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

    base = object_model.get_part("platform_base")
    oven = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    drawer = object_model.get_part("ash_drawer")
    door_hinge = object_model.get_articulation("oven_to_door")
    drawer_slide = object_model.get_articulation("oven_to_ash_drawer")

    entry_arch = oven.get_visual("entry_arch")
    door_panel = door.get_visual("door_panel")
    drawer_front = drawer.get_visual("drawer_front")
    firebox_top = base.get_visual("firebox_top")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            oven,
            door,
            axis="y",
            positive_elem=entry_arch,
            negative_elem=door_panel,
            max_gap=0.012,
            max_penetration=0.0,
            name="door closes near the oven entry face",
        )
        ctx.expect_overlap(
            oven,
            door,
            axes="xz",
            elem_a=entry_arch,
            elem_b=door_panel,
            min_overlap=0.180,
            name="door covers the front entry opening area",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
    with ctx.pose({door_hinge: math.radians(82.0)}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
    ctx.check(
        "door drops outward and downward when opened",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.12
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.08,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    ctx.expect_gap(
        base,
        drawer,
        axis="z",
        positive_elem=firebox_top,
        negative_elem=drawer_front,
        min_gap=0.030,
        name="ash drawer sits below the firebox floor",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    closed_drawer_aabb = ctx.part_element_world_aabb(drawer, elem=drawer_front)
    with ctx.pose({drawer_slide: 0.260}):
        open_drawer_pos = ctx.part_world_position(drawer)
        open_drawer_aabb = ctx.part_element_world_aabb(drawer, elem=drawer_front)
    ctx.check(
        "ash drawer slides forward from the oven front",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] < closed_drawer_pos[1] - 0.20
        and closed_drawer_aabb is not None
        and open_drawer_aabb is not None
        and open_drawer_aabb[0][1] < closed_drawer_aabb[0][1] - 0.20,
        details=(
            f"closed_pos={closed_drawer_pos}, open_pos={open_drawer_pos}, "
            f"closed_aabb={closed_drawer_aabb}, open_aabb={open_drawer_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
