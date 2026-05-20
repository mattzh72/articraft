from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_L = 0.30
BASE_W = 0.16
BASE_T = 0.022
BASE_NOTCH_W = 0.10
BASE_NOTCH_D = 0.044
BASE_SOCKET_R = 0.040
BASE_SOCKET_H = 0.012

BARREL_OUTER_R = 0.0225
BARREL_INNER_R = 0.0180
BARREL_COLLAR_R = 0.0285
BARREL_LOWER_H = 0.038
BARREL_TUBE_H = 0.490
BARREL_TOP_H = 0.020
BARREL_TOTAL_H = BARREL_LOWER_H + BARREL_TUBE_H + BARREL_TOP_H
BARREL_BORE_Z0 = 0.020
BARREL_BORE_H = 0.516
ROD_GUIDE_R = 0.0070

CLIP_PIVOT_X = 0.032
CLIP_PIVOT_Z = 0.125
CLIP_BOSS_R = 0.0065
CLIP_BOSS_T = 0.004

ROD_R = 0.0070
ROD_BELOW_JOINT = 0.360
ROD_ABOVE_JOINT = 0.120
ROD_TOTAL_L = ROD_BELOW_JOINT + ROD_ABOVE_JOINT
ROD_CENTER_Z = (ROD_ABOVE_JOINT - ROD_BELOW_JOINT) / 2.0
HANDLE_BAR_Z = 0.122
HANDLE_BAR_L = 0.250
HANDLE_BAR_R = 0.010
HANDLE_GRIP_R = 0.0135
HANDLE_GRIP_L = 0.055
HANDLE_TRAVEL = 0.240

CLIP_OUTER_R = 0.0080
CLIP_INNER_R = 0.0050
CLIP_T = 0.012
CLIP_OPEN_ANGLE = 1.05


def _centered_cylinder(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))

    front_notch = (
        cq.Workplane("XY")
        .center(0.0, (BASE_W / 2.0) - (BASE_NOTCH_D / 2.0))
        .box(BASE_NOTCH_W, BASE_NOTCH_D, BASE_T + 0.004, centered=(True, True, False))
    )
    back_notch = (
        cq.Workplane("XY")
        .center(0.0, -(BASE_W / 2.0) + (BASE_NOTCH_D / 2.0))
        .box(BASE_NOTCH_W, BASE_NOTCH_D, BASE_T + 0.004, centered=(True, True, False))
    )
    socket = cq.Workplane("XY").circle(BASE_SOCKET_R).extrude(BASE_SOCKET_H).translate((0.0, 0.0, BASE_T))

    return base.cut(front_notch).cut(back_notch).union(socket)


def _make_barrel_shape() -> cq.Workplane:
    lower_collar = _centered_cylinder(BARREL_COLLAR_R, BARREL_LOWER_H).translate(
        (0.0, 0.0, BARREL_LOWER_H / 2.0)
    )
    main_tube = _centered_cylinder(BARREL_OUTER_R, BARREL_TUBE_H).translate(
        (0.0, 0.0, BARREL_LOWER_H + (BARREL_TUBE_H / 2.0))
    )
    top_cap = _centered_cylinder(BARREL_COLLAR_R, BARREL_TOP_H).translate(
        (0.0, 0.0, BARREL_LOWER_H + BARREL_TUBE_H + (BARREL_TOP_H / 2.0))
    )
    bore = _centered_cylinder(BARREL_INNER_R, BARREL_BORE_H).translate(
        (0.0, 0.0, BARREL_BORE_Z0 + (BARREL_BORE_H / 2.0))
    )
    guide_hole = _centered_cylinder(ROD_GUIDE_R, BARREL_TOP_H + 0.004).translate(
        (0.0, 0.0, BARREL_LOWER_H + BARREL_TUBE_H + (BARREL_TOP_H / 2.0))
    )

    boss = (
        _centered_cylinder(CLIP_BOSS_R, CLIP_BOSS_T)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((CLIP_PIVOT_X, 0.0, CLIP_PIVOT_Z))
    )
    strut_start_x = BARREL_OUTER_R - 0.005
    strut_len = CLIP_PIVOT_X - strut_start_x
    rear_strut = cq.Workplane("XY").box(strut_len, 0.010, 0.020).translate(
        ((strut_start_x + (strut_len / 2.0)), 0.0, CLIP_PIVOT_Z)
    )
    mount_pad = cq.Workplane("XY").box(0.003, 0.010, 0.016).translate((0.0375, 0.0, CLIP_PIVOT_Z))

    return (
        lower_collar.union(main_tube)
        .union(top_cap)
        .cut(bore)
        .cut(guide_hole)
        .union(rear_strut)
        .union(boss)
        .union(mount_pad)
    )


def _make_handle_rod_shape() -> cq.Workplane:
    rod = _centered_cylinder(ROD_R, ROD_TOTAL_L).translate((0.0, 0.0, ROD_CENTER_Z))
    hub = _centered_cylinder(0.013, 0.030).translate((0.0, 0.0, 0.108))
    top_stub = _centered_cylinder(0.009, 0.022).translate((0.0, 0.0, 0.126))
    return rod.union(hub).union(top_stub)


def _make_handle_grip_shape() -> cq.Workplane:
    crossbar = (
        _centered_cylinder(HANDLE_BAR_R, HANDLE_BAR_L)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, 0.0, HANDLE_BAR_Z))
    )
    left_grip = (
        _centered_cylinder(HANDLE_GRIP_R, HANDLE_GRIP_L)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-(HANDLE_BAR_L / 2.0) + (HANDLE_GRIP_L / 2.0), 0.0, HANDLE_BAR_Z))
    )
    right_grip = (
        _centered_cylinder(HANDLE_GRIP_R, HANDLE_GRIP_L)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(((HANDLE_BAR_L / 2.0) - (HANDLE_GRIP_L / 2.0), 0.0, HANDLE_BAR_Z))
    )
    return crossbar.union(left_grip).union(right_grip)


def _make_clip_shape() -> cq.Workplane:
    pivot_cheek = (
        _centered_cylinder(0.006, 0.006)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.013, 0.0, 0.0))
    )
    upper_bridge = cq.Workplane("XY").box(0.016, CLIP_T, 0.018).translate((0.018, 0.0, -0.010))
    arm = cq.Workplane("XY").box(0.015, CLIP_T, 0.058).translate((0.019, 0.0, -0.040))
    hook = cq.Workplane("XY").box(0.032, CLIP_T, 0.013).translate((0.030, 0.0, -0.0715))
    return_lip = cq.Workplane("XY").box(0.010, CLIP_T, 0.020).translate((0.041, 0.0, -0.061))
    opening = cq.Workplane("XY").box(0.019, CLIP_T + 0.004, 0.036).translate((0.025, 0.0, -0.049))
    return pivot_cheek.union(upper_bridge).union(arm).union(hook).union(return_lip).cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    model.material("base_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("barrel_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rod_steel", rgba=(0.80, 0.81, 0.82, 1.0))
    model.material("handle_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("clip_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "pump_base"), material="base_steel", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + BASE_SOCKET_H)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + BASE_SOCKET_H) / 2.0)),
    )

    barrel = model.part("barrel")
    barrel.visual(mesh_from_cadquery(_make_barrel_shape(), "pump_barrel"), material="barrel_alloy", name="barrel_shell")
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_COLLAR_R, length=BARREL_TOTAL_H),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BARREL_TOTAL_H / 2.0)),
    )

    handle = model.part("handle_assembly")
    handle.visual(
        mesh_from_cadquery(_make_handle_rod_shape(), "pump_handle_rod"),
        material="rod_steel",
        name="handle_rod",
    )
    handle.visual(
        mesh_from_cadquery(_make_handle_grip_shape(), "pump_handle_grip"),
        material="handle_black",
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((HANDLE_BAR_L, 0.035, ROD_TOTAL_L + 0.020)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    clip = model.part("hose_clip")
    clip.visual(mesh_from_cadquery(_make_clip_shape(), "pump_hose_clip"), material="clip_black", name="clip_body")
    clip.inertial = Inertial.from_geometry(
        Box((0.040, CLIP_T, 0.085)),
        mass=0.08,
        origin=Origin(xyz=(0.022, 0.0, -0.038)),
    )

    model.articulation(
        "base_to_barrel",
        ArticulationType.FIXED,
        parent=base,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + BASE_SOCKET_H)),
    )
    model.articulation(
        "barrel_to_handle",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, BARREL_TOTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=HANDLE_TRAVEL, effort=120.0, velocity=0.80),
    )
    model.articulation(
        "barrel_to_hose_clip",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=clip,
        origin=Origin(xyz=(CLIP_PIVOT_X, 0.0, CLIP_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=2.0, velocity=1.5),
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
    barrel = object_model.get_part("barrel")
    handle = object_model.get_part("handle_assembly")
    clip = object_model.get_part("hose_clip")
    handle_slide = object_model.get_articulation("barrel_to_handle")
    clip_pivot = object_model.get_articulation("barrel_to_hose_clip")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (base, barrel, handle, clip)),
        details=f"parts={[part.name for part in (base, barrel, handle, clip)]}",
    )
    ctx.allow_overlap(
        barrel,
        handle,
        elem_a="barrel_shell",
        elem_b="handle_rod",
        reason="The piston rod is intentionally represented as a simplified sliding member inside the barrel shell mesh.",
    )
    ctx.expect_contact(base, barrel, name="barrel is seated on the base")
    ctx.expect_origin_distance(
        clip,
        barrel,
        axes="x",
        min_dist=0.029,
        max_dist=0.035,
        name="hose clip pivots from the side of the barrel",
    )
    ctx.expect_contact(clip, barrel, name="hose clip is mounted on the barrel pivot")
    ctx.expect_within(
        handle,
        barrel,
        axes="xy",
        inner_elem="handle_rod",
        outer_elem="barrel_shell",
        margin=0.006,
        name="rod stays centered in the barrel at rest",
    )
    ctx.expect_overlap(
        handle,
        barrel,
        axes="z",
        elem_a="handle_rod",
        elem_b="barrel_shell",
        min_overlap=0.32,
        name="closed handle rod remains deeply inserted",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        ctx.expect_within(
            handle,
            barrel,
            axes="xy",
            inner_elem="handle_rod",
            outer_elem="barrel_shell",
            margin=0.006,
            name="rod stays centered when extended",
        )
        ctx.expect_overlap(
            handle,
            barrel,
            axes="z",
            elem_a="handle_rod",
            elem_b="barrel_shell",
            min_overlap=0.10,
            name="extended handle rod still retains insertion",
        )
        raised_handle_pos = ctx.part_world_position(handle)

    ctx.check(
        "handle assembly slides upward",
        rest_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > rest_handle_pos[2] + 0.20,
        details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
    )

    rest_clip_aabb = ctx.part_element_world_aabb(clip, elem="clip_body")
    with ctx.pose({clip_pivot: CLIP_OPEN_ANGLE}):
        open_clip_aabb = ctx.part_element_world_aabb(clip, elem="clip_body")

    rest_clip_center_x = None
    if rest_clip_aabb is not None:
        rest_clip_center_x = (rest_clip_aabb[0][0] + rest_clip_aabb[1][0]) / 2.0
    open_clip_center_x = None
    if open_clip_aabb is not None:
        open_clip_center_x = (open_clip_aabb[0][0] + open_clip_aabb[1][0]) / 2.0

    ctx.check(
        "hose clip swings outward when opened",
        rest_clip_center_x is not None
        and open_clip_center_x is not None
        and open_clip_center_x > rest_clip_center_x + 0.012,
        details=f"rest_x={rest_clip_center_x}, open_x={open_clip_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
