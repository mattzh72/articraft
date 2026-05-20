from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """A soft upholstered rounded rectangle, authored in meters."""
    solid = cq.Workplane("XY").box(size[0], size[1], size[2]).edges().fillet(radius)
    return mesh_from_cadquery(solid, name, tolerance=0.0015, angular_tolerance=0.16)


def _matte_black(model: ArticulatedObject) -> Material:
    return model.material("matte_black_plastic", rgba=(0.025, 0.025, 0.023, 1.0))


def _add_caster_wheel_visuals(part, prefix: str, rubber, dark_metal, chrome) -> None:
    tire = TireGeometry(
        0.055,
        0.040,
        inner_radius=0.034,
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.004, radius=0.0025),
    )
    rim = WheelGeometry(
        0.035,
        0.034,
        rim=WheelRim(inner_radius=0.020, flange_height=0.003, flange_thickness=0.002),
        hub=WheelHub(radius=0.014, width=0.046, cap_style="domed"),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
        bore=WheelBore(style="round", diameter=0.008),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=rubber, name="tire")
    part.visual(mesh_from_geometry(rim, f"{prefix}_rim"), material=dark_metal, name="wheel_rim")
    part.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="hub_cap",
    )


def _add_caster_fork_visuals(part, dark_metal, chrome) -> None:
    part.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=chrome,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_metal,
        name="swivel_cap",
    )
    part.visual(
        Box((0.116, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_metal,
        name="fork_bridge",
    )
    for side, x in (("a", -0.048), ("b", 0.048)):
        part.visual(
            Box((0.012, 0.055, 0.115)),
            origin=Origin(xyz=(x, 0.0, -0.0875)),
            material=dark_metal,
            name=f"fork_cheek_{side}",
        )
    part.visual(
        Cylinder(radius=0.0065, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, -0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="axle",
    )


def _add_armrest_visuals(part, prefix: str, leather, dark_metal, chrome) -> None:
    part.visual(
        _rounded_box_mesh((0.455, 0.086, 0.044), 0.018, f"{prefix}_pad"),
        origin=Origin(xyz=(0.235, 0.0, 0.046)),
        material=leather,
        name="padded_top",
    )
    part.visual(
        Box((0.430, 0.033, 0.024)),
        origin=Origin(xyz=(0.230, 0.0, 0.012)),
        material=dark_metal,
        name="under_rail",
    )
    part.visual(
        Box((0.034, 0.034, 0.165)),
        origin=Origin(xyz=(0.430, 0.0, -0.070)),
        material=dark_metal,
        name="front_drop",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.106),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_flip_up_office_chair")

    black_plastic = _matte_black(model)
    leather = model.material("black_grained_leather", rgba=(0.045, 0.040, 0.035, 1.0))
    seam = model.material("slightly_glossy_seam", rgba=(0.010, 0.010, 0.010, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.74, 0.75, 0.76, 1.0))
    dark_metal = model.material("dark_powder_coat", rgba=(0.11, 0.11, 0.11, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.018, 0.018, 0.017, 1.0))

    base = model.part("star_base")
    base.visual(
        Cylinder(radius=0.092, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=black_plastic,
        name="central_hub",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.195),
        origin=Origin(xyz=(0.0, 0.0, 0.2175)),
        material=dark_metal,
        name="lower_sleeve",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=chrome,
        name="top_clip_ring",
    )
    caster_mounts: list[tuple[float, float, float]] = []
    for index in range(5):
        angle = pi / 2.0 + 2.0 * pi * index / 5.0
        mid_r = 0.335
        end_r = 0.570
        base.visual(
            Box((0.510, 0.082, 0.045)),
            origin=Origin(
                xyz=(cos(angle) * mid_r, sin(angle) * mid_r, 0.185),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_plastic,
            name=f"star_spoke_{index}",
        )
        base.visual(
            Cylinder(radius=0.052, length=0.012),
            origin=Origin(
                xyz=(cos(angle) * end_r, sin(angle) * end_r, 0.166),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"caster_boss_{index}",
        )
        caster_mounts.append((cos(angle) * end_r, sin(angle) * end_r, 0.160))

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=0.028, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=chrome,
        name="inner_piston",
    )
    gas_column.visual(
        Cylinder(radius=0.038, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="seat_clip_collar",
    )

    seat = model.part("seat")
    seat.visual(
        _rounded_box_mesh((0.585, 0.545, 0.112), 0.040, "seat_cushion"),
        origin=Origin(xyz=(0.030, 0.0, 0.078)),
        material=leather,
        name="cushion",
    )
    seat.visual(
        _rounded_box_mesh((0.355, 0.310, 0.050), 0.012, "underseat_mechanism"),
        origin=Origin(xyz=(-0.005, 0.0, 0.000)),
        material=dark_metal,
        name="underseat_plate",
    )
    seat.visual(
        Cylinder(radius=0.048, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=dark_metal,
        name="seat_socket",
    )
    seat.visual(
        Box((0.004, 0.500, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.137)),
        material=seam,
        name="seat_center_seam",
    )
    seat.visual(
        Box((0.470, 0.004, 0.006)),
        origin=Origin(xyz=(0.055, -0.112, 0.138)),
        material=seam,
        name="seat_side_seam_0",
    )
    seat.visual(
        Box((0.470, 0.004, 0.006)),
        origin=Origin(xyz=(0.055, 0.112, 0.138)),
        material=seam,
        name="seat_side_seam_1",
    )
    seat.visual(
        Box((0.095, 0.035, 0.102)),
        origin=Origin(xyz=(-0.300, -0.235, 0.120)),
        material=dark_metal,
        name="back_yoke_0",
    )
    seat.visual(
        Box((0.095, 0.035, 0.102)),
        origin=Origin(xyz=(-0.300, 0.235, 0.120)),
        material=dark_metal,
        name="back_yoke_1",
    )
    seat.visual(
        Box((0.160, 0.030, 0.050)),
        origin=Origin(xyz=(-0.220, -0.235, 0.055)),
        material=dark_metal,
        name="back_yoke_foot_0",
    )
    seat.visual(
        Box((0.160, 0.030, 0.050)),
        origin=Origin(xyz=(-0.220, 0.235, 0.055)),
        material=dark_metal,
        name="back_yoke_foot_1",
    )
    for side, y in (("right", -0.340), ("left", 0.340)):
        seat.visual(
            Box((0.205, 0.230, 0.035)),
            origin=Origin(xyz=(-0.155, y * 0.75, 0.035)),
            material=dark_metal,
            name=f"{side}_arm_side_brace",
        )
        seat.visual(
            Box((0.054, 0.052, 0.200)),
            origin=Origin(xyz=(-0.220, y, 0.105)),
            material=dark_metal,
            name=f"{side}_rear_arm_post",
        )
        seat.visual(
            Cylinder(radius=0.018, length=0.068),
            origin=Origin(xyz=(-0.220, y, 0.213), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"{side}_arm_hinge_boss",
        )

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_box_mesh((0.105, 0.515, 0.785), 0.050, "high_back_cushion"),
        origin=Origin(xyz=(-0.055, 0.0, 0.470), rpy=(0.0, -0.055, 0.0)),
        material=leather,
        name="high_back_pad",
    )
    backrest.visual(
        _rounded_box_mesh((0.052, 0.475, 0.165), 0.035, "head_pillow"),
        origin=Origin(xyz=(-0.105, 0.0, 0.805), rpy=(0.0, -0.070, 0.0)),
        material=leather,
        name="head_pillow",
    )
    backrest.visual(
        _rounded_box_mesh((0.042, 0.430, 0.120), 0.028, "lumbar_pillow"),
        origin=Origin(xyz=(-0.115, 0.0, 0.405), rpy=(0.0, -0.045, 0.0)),
        material=leather,
        name="lumbar_pad",
    )
    for y in (-0.168, 0.168):
        backrest.visual(
            Box((0.006, 0.004, 0.610)),
            origin=Origin(xyz=(-0.120, y, 0.520), rpy=(0.0, -0.060, 0.0)),
            material=seam,
            name=f"back_vertical_seam_{0 if y < 0 else 1}",
        )
    backrest.visual(
        Cylinder(radius=0.024, length=0.435),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="recline_barrel",
    )
    backrest.visual(
        Box((0.060, 0.055, 0.250)),
        origin=Origin(xyz=(-0.040, -0.140, 0.140), rpy=(0.0, -0.060, 0.0)),
        material=dark_metal,
        name="back_strut_0",
    )
    backrest.visual(
        Box((0.060, 0.055, 0.250)),
        origin=Origin(xyz=(-0.040, 0.140, 0.140), rpy=(0.0, -0.060, 0.0)),
        material=dark_metal,
        name="back_strut_1",
    )

    left_armrest = model.part("left_armrest")
    _add_armrest_visuals(left_armrest, "left_armrest", leather, dark_metal, chrome)
    right_armrest = model.part("right_armrest")
    _add_armrest_visuals(right_armrest, "right_armrest", leather, dark_metal, chrome)

    model.articulation(
        "gas_lift_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gas_column,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.12, lower=0.0, upper=0.090),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.300, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=0.48),
    )
    model.articulation(
        "left_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=left_armrest,
        origin=Origin(xyz=(-0.220, 0.340, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "right_arm_flip",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=right_armrest,
        origin=Origin(xyz=(-0.220, -0.340, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.85),
    )

    for index, (x, y, z) in enumerate(caster_mounts):
        fork = model.part(f"caster_fork_{index}")
        _add_caster_fork_visuals(fork, dark_metal, chrome)
        wheel = model.part(f"caster_wheel_{index}")
        _add_caster_wheel_visuals(wheel, f"caster_wheel_{index}", rubber, dark_metal, chrome)
        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )
        model.articulation(
            f"caster_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.105)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("star_base")
    gas = object_model.get_part("gas_column")
    seat = object_model.get_part("seat")
    back = object_model.get_part("backrest")
    left_arm = object_model.get_part("left_armrest")
    right_arm = object_model.get_part("right_armrest")
    lift = object_model.get_articulation("gas_lift_slide")
    back_recline = object_model.get_articulation("back_recline")
    left_flip = object_model.get_articulation("left_arm_flip")
    right_flip = object_model.get_articulation("right_arm_flip")

    ctx.allow_overlap(
        base,
        gas,
        elem_a="lower_sleeve",
        elem_b="inner_piston",
        reason="The gas-lift piston is intentionally retained inside the lower pedestal sleeve.",
    )
    ctx.allow_overlap(
        base,
        gas,
        elem_a="central_hub",
        elem_b="inner_piston",
        reason="The piston passes through the base hub socket as part of the pedestal stack.",
    )
    ctx.allow_overlap(
        base,
        gas,
        elem_a="top_clip_ring",
        elem_b="inner_piston",
        reason="The pedestal's top retaining ring is represented as a solid guide around the piston.",
    )
    ctx.allow_overlap(
        seat,
        gas,
        elem_a="seat_socket",
        elem_b="inner_piston",
        reason="The upper gas piston is clipped into the under-seat socket so the chair remains stacked.",
    )
    ctx.allow_overlap(
        seat,
        gas,
        elem_a="seat_socket",
        elem_b="seat_clip_collar",
        reason="The upper collar is seated inside the under-seat receiver socket.",
    )
    ctx.allow_overlap(
        seat,
        gas,
        elem_a="underseat_plate",
        elem_b="inner_piston",
        reason="The under-seat mechanism plate is simplified as solid around the gas lift pass-through.",
    )
    for side, arm in (("left", left_arm), ("right", right_arm)):
        ctx.allow_overlap(
            seat,
            arm,
            elem_a=f"{side}_arm_hinge_boss",
            elem_b="hinge_barrel",
            reason="The flip-up armrest hinge barrel is captured in the rear bracket boss.",
        )
        ctx.allow_overlap(
            seat,
            arm,
            elem_a=f"{side}_arm_hinge_boss",
            elem_b="hinge_pin",
            reason="The hinge pin passes through the rear bracket boss.",
        )
    for index in range(5):
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="axle",
            elem_b="wheel_rim",
            reason="Each caster axle is intentionally captured through the wheel hub.",
        )
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="axle",
            elem_b="hub_cap",
            reason="Each caster axle visibly passes through the domed hub cap.",
        )

    ctx.expect_within(
        gas,
        base,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="lower_sleeve",
        margin=0.004,
        name="gas piston centered in pedestal sleeve",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="lower_sleeve",
        min_overlap=0.150,
        name="collapsed gas piston remains inserted in base sleeve",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="central_hub",
        min_overlap=0.030,
        name="gas piston passes through base hub socket",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="top_clip_ring",
        min_overlap=0.010,
        name="gas piston passes through pedestal top clip ring",
    )
    ctx.expect_within(
        gas,
        seat,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="seat_socket",
        margin=0.006,
        name="gas piston centered in under-seat socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="inner_piston",
        elem_b="seat_socket",
        min_overlap=0.035,
        name="gas piston enters under-seat socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="inner_piston",
        elem_b="underseat_plate",
        min_overlap=0.015,
        name="gas piston passes through under-seat mechanism plate",
    )
    ctx.expect_within(
        gas,
        seat,
        axes="xy",
        inner_elem="seat_clip_collar",
        outer_elem="seat_socket",
        margin=0.004,
        name="seat clip collar is centered in socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="seat_clip_collar",
        elem_b="seat_socket",
        min_overlap=0.020,
        name="seat clip collar is retained by socket",
    )

    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.090}):
        raised_seat = ctx.part_world_position(seat)
        ctx.expect_overlap(
            gas,
            base,
            axes="z",
            elem_a="inner_piston",
            elem_b="lower_sleeve",
            min_overlap=0.060,
            name="raised gas piston remains inserted in sleeve",
        )
        ctx.expect_overlap(
            gas,
            seat,
            axes="z",
            elem_a="inner_piston",
            elem_b="seat_socket",
            min_overlap=0.035,
            name="raised gas piston stays clipped into seat socket",
        )
    ctx.check(
        "gas lift raises the whole seat stack",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.075,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    rest_back_aabb = ctx.part_world_aabb(back)
    with ctx.pose({back_recline: 0.48}):
        reclined_back_aabb = ctx.part_world_aabb(back)
    ctx.check(
        "backrest reclines rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.070,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )

    rest_left_aabb = ctx.part_world_aabb(left_arm)
    rest_right_aabb = ctx.part_world_aabb(right_arm)
    with ctx.pose({left_flip: 1.85, right_flip: 1.85}):
        up_left_aabb = ctx.part_world_aabb(left_arm)
        up_right_aabb = ctx.part_world_aabb(right_arm)
    ctx.check(
        "left armrest flips upward",
        rest_left_aabb is not None and up_left_aabb is not None and up_left_aabb[1][2] > rest_left_aabb[1][2] + 0.28,
        details=f"rest={rest_left_aabb}, up={up_left_aabb}",
    )
    ctx.check(
        "right armrest flips upward",
        rest_right_aabb is not None and up_right_aabb is not None and up_right_aabb[1][2] > rest_right_aabb[1][2] + 0.28,
        details=f"rest={rest_right_aabb}, up={up_right_aabb}",
    )

    for index in range(5):
        ctx.expect_contact(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="axle",
            elem_b="wheel_rim",
            contact_tol=0.0005,
            name=f"caster {index} axle passes through wheel hub",
        )
        ctx.expect_contact(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="axle",
            elem_b="hub_cap",
            contact_tol=0.0005,
            name=f"caster {index} axle reaches hub cap",
        )

    for side, arm in (("left", left_arm), ("right", right_arm)):
        ctx.expect_contact(
            seat,
            arm,
            elem_a=f"{side}_arm_hinge_boss",
            elem_b="hinge_barrel",
            contact_tol=0.0005,
            name=f"{side} flip-up arm hinge is seated in rear bracket",
        )

    return ctx.report()


object_model = build_object_model()
