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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _handle_section(
    x: float,
    *,
    width: float,
    height: float,
    z_center: float,
    side_chamfer: float = 0.0028,
    top_chamfer: float = 0.0025,
    bottom_chamfer: float = 0.0018,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    z_min = z_center - half_h
    z_max = z_center + half_h
    return [
        (x, -half_w + side_chamfer, z_min),
        (x, half_w - side_chamfer, z_min),
        (x, half_w, z_min + bottom_chamfer),
        (x, half_w, z_max - top_chamfer),
        (x, half_w - side_chamfer, z_max),
        (x, -half_w + side_chamfer, z_max),
        (x, -half_w, z_max - top_chamfer),
        (x, -half_w, z_min + bottom_chamfer),
    ]


def _blade_profile() -> list[tuple[float, float]]:
    return [
        (0.060, 0.000),
        (0.082, 0.000),
        (0.104, 0.0045),
        (0.103, 0.0060),
        (0.088, 0.0072),
        (0.081, 0.0100),
        (0.060, 0.0100),
    ]


def _clip_profile() -> list[tuple[float, float]]:
    return [
        (0.012, -0.006),
        (0.018, 0.000),
        (0.076, 0.000),
        (0.088, -0.003),
        (0.086, -0.009),
        (0.079, -0.016),
        (0.030, -0.019),
        (0.012, -0.019),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_knife")

    body_dark = model.material("body_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    grip_yellow = model.material("grip_yellow", rgba=(0.88, 0.72, 0.16, 1.0))
    slider_black = model.material("slider_black", rgba=(0.10, 0.10, 0.11, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    clip_steel = model.material("clip_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    handle = model.part("handle")

    handle_geom = section_loft(
        [
            _handle_section(-0.078, width=0.034, height=0.017, z_center=-0.0015),
            _handle_section(-0.030, width=0.036, height=0.019, z_center=-0.0015),
            _handle_section(0.015, width=0.035, height=0.020, z_center=-0.0012),
            _handle_section(0.048, width=0.029, height=0.016, z_center=-0.0010),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "utility_knife_handle_shell"),
        material=body_dark,
        name="body_shell",
    )
    handle.visual(
        Box((0.102, 0.0085, 0.0035)),
        origin=Origin(xyz=(-0.006, -0.013, 0.00875)),
        material=body_dark,
        name="left_spine_rail",
    )
    handle.visual(
        Box((0.102, 0.0085, 0.0035)),
        origin=Origin(xyz=(-0.006, 0.013, 0.00875)),
        material=body_dark,
        name="right_spine_rail",
    )
    handle.visual(
        Box((0.070, 0.0018, 0.011)),
        origin=Origin(xyz=(-0.010, -0.0186, -0.0005)),
        material=grip_yellow,
        name="left_grip_panel",
    )
    handle.visual(
        Box((0.030, 0.026, 0.006)),
        origin=Origin(xyz=(0.062, 0.000, -0.0010)),
        material=body_dark,
        name="nose_chin",
    )
    handle.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.062, -0.010, 0.006)),
        material=body_dark,
        name="nose_cheek_left",
    )
    handle.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.062, 0.010, 0.006)),
        material=body_dark,
        name="nose_cheek_right",
    )
    handle.visual(
        Box((0.014, 0.004, 0.012)),
        origin=Origin(xyz=(-0.054, 0.018, 0.004)),
        material=body_dark,
        name="clip_mount_pad",
    )
    handle.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(
            xyz=(-0.049, 0.0202, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_dark,
        name="clip_lug_rear",
    )
    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.050, 0.0065, 0.0026)),
        origin=Origin(xyz=(0.025, 0.000, 0.0013)),
        material=slider_black,
        name="carriage_shoe",
    )
    blade_carriage.visual(
        Box((0.020, 0.013, 0.007)),
        origin=Origin(xyz=(0.024, 0.000, 0.0058)),
        material=slider_black,
        name="slider_button",
    )
    for rib_index, rib_x in enumerate((0.019, 0.024, 0.029), start=1):
        blade_carriage.visual(
            Box((0.0020, 0.013, 0.0015)),
            origin=Origin(xyz=(rib_x, 0.000, 0.0100)),
            material=slider_black,
            name=f"button_rib_{rib_index}",
        )
    blade_carriage.visual(
        Box((0.022, 0.004, 0.0032)),
        origin=Origin(xyz=(0.051, 0.000, 0.0016)),
        material=slider_black,
        name="blade_stem",
    )
    blade_carriage.visual(
        mesh_from_geometry(
            ExtrudeGeometry(_blade_profile(), 0.0012),
            "utility_knife_blade",
        ),
        origin=Origin(xyz=(0.000, 0.000, -0.0020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_steel,
        name="blade_shell",
    )

    belt_clip = model.part("belt_clip")
    belt_clip.visual(
        mesh_from_geometry(
            ExtrudeGeometry(_clip_profile(), 0.0012),
            "utility_knife_belt_clip",
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clip_steel,
        name="clip_strip",
    )
    belt_clip.visual(
        Box((0.014, 0.0012, 0.004)),
        origin=Origin(xyz=(0.015, 0.000, -0.0020)),
        material=clip_steel,
        name="clip_neck",
    )
    belt_clip.visual(
        Cylinder(radius=0.0022, length=0.014),
        origin=Origin(xyz=(0.007, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clip_steel,
        name="clip_barrel",
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carriage,
        origin=Origin(xyz=(-0.030, 0.000, 0.0090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "handle_to_belt_clip",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=belt_clip,
        origin=Origin(xyz=(-0.046, 0.0202, 0.010)),
        # Closed clip hangs in local -Z; +X rotation lifts the lower edge outboard.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
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

    handle = object_model.get_part("handle")
    blade_carriage = object_model.get_part("blade_carriage")
    belt_clip = object_model.get_part("belt_clip")
    slide_joint = object_model.get_articulation("handle_to_blade_carriage")
    clip_hinge = object_model.get_articulation("handle_to_belt_clip")

    ctx.expect_overlap(
        blade_carriage,
        handle,
        axes="x",
        elem_a="carriage_shoe",
        elem_b="body_shell",
        min_overlap=0.045,
        name="carriage shoe stays engaged with the handle at rest",
    )
    ctx.expect_gap(
        belt_clip,
        handle,
        axis="y",
        positive_elem="clip_strip",
        negative_elem="body_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="closed belt clip sits just off the side wall",
    )
    ctx.expect_overlap(
        belt_clip,
        handle,
        axes="xz",
        elem_a="clip_strip",
        elem_b="body_shell",
        min_overlap=0.010,
        name="closed belt clip tracks the handle side silhouette",
    )

    rest_carriage_pos = ctx.part_world_position(blade_carriage)
    slide_upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else None
    if slide_upper is not None:
        with ctx.pose({slide_joint: slide_upper}):
            extended_carriage_pos = ctx.part_world_position(blade_carriage)
            ctx.expect_overlap(
                blade_carriage,
                handle,
                axes="x",
                elem_a="carriage_shoe",
                elem_b="body_shell",
                min_overlap=0.018,
                name="extended carriage still retains insertion in the handle",
            )
        ctx.check(
            "blade carriage extends forward",
            rest_carriage_pos is not None
            and extended_carriage_pos is not None
            and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.030,
            details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
        )

    closed_clip_aabb = ctx.part_element_world_aabb(belt_clip, elem="clip_strip")
    clip_upper = clip_hinge.motion_limits.upper if clip_hinge.motion_limits is not None else None
    if clip_upper is not None:
        with ctx.pose({clip_hinge: clip_upper}):
            open_clip_aabb = ctx.part_element_world_aabb(belt_clip, elem="clip_strip")
        ctx.check(
            "belt clip swings outward from the handle side",
            closed_clip_aabb is not None
            and open_clip_aabb is not None
            and open_clip_aabb[1][1] > closed_clip_aabb[1][1] + 0.010,
            details=f"closed={closed_clip_aabb}, open={open_clip_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
