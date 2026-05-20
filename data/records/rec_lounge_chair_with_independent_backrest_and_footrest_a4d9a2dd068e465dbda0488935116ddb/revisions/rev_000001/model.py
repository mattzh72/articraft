from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_loop(
    width: float,
    height: float,
    radius: float,
    z: float,
    *,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
    offset: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    scaled_width = width * scale_x
    scaled_height = height * scale_y
    corner = min(radius, scaled_width * 0.49, scaled_height * 0.49)
    ox, oy = offset
    return [
        (x + ox, y + oy, z)
        for x, y in rounded_rect_profile(
            scaled_width,
            scaled_height,
            corner,
            corner_segments=8,
        )
    ]


def _cushion_mesh(
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
    *,
    top_offset: tuple[float, float] = (0.0, 0.0),
):
    lower = _rounded_loop(length, width, corner_radius, 0.0)
    mid = _rounded_loop(
        length,
        width,
        corner_radius,
        thickness * 0.52,
        scale_x=0.975,
        scale_y=0.985,
        offset=(top_offset[0] * 0.45, top_offset[1] * 0.45),
    )
    upper = _rounded_loop(
        length,
        width,
        corner_radius,
        thickness,
        scale_x=0.93,
        scale_y=0.95,
        offset=top_offset,
    )
    return LoftGeometry([lower, mid, upper], cap=True, closed=True)


def _vertical_pad_mesh(
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
):
    return _cushion_mesh(
        length=height,
        width=width,
        thickness=thickness,
        corner_radius=corner_radius,
        top_offset=(0.0, 0.0),
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_pedestal_lounge_chair")

    base_metal = model.material("base_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.56, 0.58, 0.61, 1.0))
    shell_charcoal = model.material("shell_charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    upholstery = model.material("warm_taupe", rgba=(0.76, 0.72, 0.66, 1.0))
    upholstery_shadow = model.material("shadow_taupe", rgba=(0.60, 0.56, 0.51, 1.0))

    seat_shell_mesh = _mesh(
        "seat_shell",
        _cushion_mesh(
            length=0.66,
            width=0.78,
            thickness=0.034,
            corner_radius=0.11,
            top_offset=(-0.010, 0.0),
        ),
    )
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        _cushion_mesh(
            length=0.62,
            width=0.74,
            thickness=0.095,
            corner_radius=0.105,
            top_offset=(-0.014, 0.0),
        ),
    )
    back_pad_mesh = _mesh(
        "split_back_pad",
        _vertical_pad_mesh(
            width=0.22,
            height=0.56,
            thickness=0.060,
            corner_radius=0.060,
        ),
    )
    lumbar_bridge_mesh = _mesh(
        "lumbar_bridge",
        _vertical_pad_mesh(
            width=0.56,
            height=0.14,
            thickness=0.040,
            corner_radius=0.045,
        ),
    )
    leg_rest_mesh = _mesh(
        "leg_rest_panel",
        _cushion_mesh(
            length=0.36,
            width=0.50,
            thickness=0.055,
            corner_radius=0.085,
            top_offset=(-0.004, 0.0),
        ),
    )
    pedestal_shell_mesh = _mesh(
        "pedestal_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.31, 0.0),
                (0.31, 0.018),
                (0.28, 0.030),
                (0.18, 0.043),
                (0.105, 0.060),
                (0.097, 0.245),
                (0.132, 0.270),
                (0.165, 0.286),
                (0.0, 0.286),
            ],
            segments=72,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        pedestal_shell_mesh,
        material=base_metal,
        name="pedestal_shell",
    )
    pedestal_base.visual(
        Cylinder(radius=0.115, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=satin_alloy,
        name="swivel_collar",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=0.294),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_alloy,
        name="swivel_plate",
    )
    seat.visual(
        Box((0.44, 0.26, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, 0.040)),
        material=base_metal,
        name="underframe_block",
    )
    seat.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=upholstery_shadow,
        name="seat_shell",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=upholstery,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.030, 0.58, 0.060)),
        origin=Origin(xyz=(0.305, 0.0, 0.045)),
        material=shell_charcoal,
        name="front_apron",
    )
    seat.visual(
        Box((0.084, 0.44, 0.034)),
        origin=Origin(xyz=(0.182, 0.0, 0.008)),
        material=base_metal,
        name="leg_hinge_housing",
    )
    seat.visual(
        Box((0.056, 0.036, 0.082)),
        origin=Origin(xyz=(0.240, 0.186, -0.006)),
        material=base_metal,
        name="leg_bracket_left",
    )
    seat.visual(
        Box((0.056, 0.036, 0.082)),
        origin=Origin(xyz=(0.240, -0.186, -0.006)),
        material=base_metal,
        name="leg_bracket_right",
    )
    seat.visual(
        Box((0.040, 0.54, 0.070)),
        origin=Origin(xyz=(-0.282, 0.0, 0.060)),
        material=shell_charcoal,
        name="back_hinge_beam",
    )
    seat.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(-0.260, 0.175, 0.055)),
        material=shell_charcoal,
        name="back_bracket_left",
    )
    seat.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(-0.260, -0.175, 0.055)),
        material=shell_charcoal,
        name="back_bracket_right",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.78, 0.82, 0.19)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.020, length=0.360),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=base_metal,
        name="back_hinge_barrel",
    )
    backrest.visual(
        Box((0.045, 0.080, 0.180)),
        origin=Origin(xyz=(-0.045, 0.0, 0.085)),
        material=shell_charcoal,
        name="back_spine",
    )
    backrest.visual(
        lumbar_bridge_mesh,
        origin=Origin(xyz=(-0.032, 0.0, 0.160)),
        material=upholstery_shadow,
        name="lumbar_bridge",
    )
    backrest.visual(
        back_pad_mesh,
        origin=Origin(xyz=(-0.050, 0.160, 0.390)),
        material=upholstery,
        name="left_back_pad",
    )
    backrest.visual(
        back_pad_mesh,
        origin=Origin(xyz=(-0.050, -0.160, 0.390)),
        material=upholstery,
        name="right_back_pad",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.14, 0.58, 0.70)),
        mass=9.0,
        origin=Origin(xyz=(-0.045, 0.0, 0.330)),
    )

    leg_rest = model.part("leg_rest")
    leg_rest.visual(
        Cylinder(radius=0.016, length=0.336),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=base_metal,
        name="leg_hinge_barrel",
    )
    leg_rest.visual(
        Box((0.170, 0.068, 0.032)),
        origin=Origin(xyz=(0.085, 0.134, -0.024)),
        material=shell_charcoal,
        name="link_left",
    )
    leg_rest.visual(
        Box((0.170, 0.068, 0.032)),
        origin=Origin(xyz=(0.085, -0.134, -0.024)),
        material=shell_charcoal,
        name="link_right",
    )
    leg_rest.visual(
        Box((0.220, 0.360, 0.020)),
        origin=Origin(xyz=(0.155, 0.0, -0.048)),
        material=shell_charcoal,
        name="panel_support",
    )
    leg_rest.visual(
        leg_rest_mesh,
        origin=Origin(xyz=(0.210, 0.0, -0.056), rpy=(0.0, -0.05, 0.0)),
        material=upholstery,
        name="leg_rest_panel",
    )
    leg_rest.inertial = Inertial.from_geometry(
        Box((0.40, 0.52, 0.08)),
        mass=5.5,
        origin=Origin(xyz=(0.200, 0.0, -0.055)),
    )

    model.articulation(
        "pedestal_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.298, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=0.75,
        ),
    )
    model.articulation(
        "seat_to_leg_rest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=leg_rest,
        origin=Origin(xyz=(0.240, 0.0, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=0.90,
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
    pedestal_base = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    leg_rest = object_model.get_part("leg_rest")
    swivel = object_model.get_articulation("pedestal_swivel")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    leg_hinge = object_model.get_articulation("seat_to_leg_rest")

    seat.get_visual("seat_cushion")
    backrest.get_visual("left_back_pad")
    backrest.get_visual("right_back_pad")
    leg_rest.get_visual("leg_rest_panel")

    ctx.check(
        "pedestal swivel uses a vertical axis",
        swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "backrest hinge uses a transverse axis",
        back_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={back_hinge.axis}",
    )
    ctx.check(
        "leg-rest hinge uses a transverse axis",
        leg_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={leg_hinge.axis}",
    )

    with ctx.pose({swivel: 0.0, back_hinge: 0.0, leg_hinge: 0.0}):
        ctx.expect_gap(
            seat,
            pedestal_base,
            axis="z",
            positive_elem="swivel_plate",
            negative_elem="swivel_collar",
            min_gap=0.0,
            max_gap=0.012,
            name="seat sits just above the pedestal bearing",
        )
        ctx.expect_overlap(
            seat,
            pedestal_base,
            axes="xy",
            min_overlap=0.20,
            name="seat remains centered over the disc pedestal",
        )
        ctx.expect_gap(
            seat,
            leg_rest,
            axis="z",
            positive_elem="seat_cushion",
            negative_elem="leg_rest_panel",
            min_gap=0.006,
            max_gap=0.090,
            name="stowed leg rest stays beneath the seat cushion",
        )
        ctx.expect_within(
            leg_rest,
            seat,
            axes="y",
            margin=0.030,
            name="leg rest stays within the chair width when stowed",
        )

        rest_back_aabb = ctx.part_world_aabb(backrest)
        rest_leg_aabb = ctx.part_world_aabb(leg_rest)

    with ctx.pose({back_hinge: 0.55, leg_hinge: 0.78}):
        reclined_back_aabb = ctx.part_world_aabb(backrest)
        extended_leg_aabb = ctx.part_world_aabb(leg_rest)
        ctx.expect_within(
            leg_rest,
            seat,
            axes="y",
            margin=0.030,
            name="leg rest stays centered when extended",
        )

    ctx.check(
        "backrest reclines rearward",
        (
            rest_back_aabb is not None
            and reclined_back_aabb is not None
            and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.10
        ),
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )
    ctx.check(
        "leg rest swings upward and forward",
        (
            rest_leg_aabb is not None
            and extended_leg_aabb is not None
            and extended_leg_aabb[1][2] > rest_leg_aabb[1][2] + 0.20
        ),
        details=f"rest={rest_leg_aabb}, extended={extended_leg_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
