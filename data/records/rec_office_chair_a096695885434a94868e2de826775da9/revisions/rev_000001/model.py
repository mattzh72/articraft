from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CapsuleGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _merge(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geom in geometries:
        merged.merge(geom)
    return merged


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*xyz)


def _local_box(
    size: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    yaw: float,
) -> MeshGeometry:
    return BoxGeometry(size).translate(*local_xyz).rotate_z(yaw)


def _capsule_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> MeshGeometry:
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 2.0 * radius:
        return CylinderGeometry(radius, max(length, radius)).translate(
            (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
        )

    geom = CapsuleGeometry(radius, length - 2.0 * radius)
    # Local capsule axis starts on +Z.  Rotate it to the requested vector.
    xy = math.sqrt(dx * dx + dy * dy)
    geom.rotate_y(math.atan2(xy, dz))
    geom.rotate_z(math.atan2(dy, dx))
    geom.translate((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return geom


def _rounded_slab(
    size_xy: tuple[float, float],
    height: float,
    radius: float,
    xyz: tuple[float, float, float],
) -> MeshGeometry:
    profile = rounded_rect_profile(size_xy[0], size_xy[1], radius, corner_segments=10)
    return ExtrudeGeometry(profile, height, center=True).translate(*xyz)


def _base_frame_geometry() -> MeshGeometry:
    geom = MeshGeometry()

    # Five-star base hub and a hollow gas-lift sleeve.  The sleeve is authored as
    # a real thin-walled shell so the sliding mast has clearance instead of
    # hiding in a solid cylinder.
    geom.merge(
        LatheGeometry.from_shell_profiles(
            [(0.105, 0.070), (0.105, 0.140)],
            [(0.047, 0.074), (0.047, 0.136)],
            segments=64,
            start_cap="round",
            end_cap="round",
            lip_samples=4,
        )
    )
    geom.merge(
        LatheGeometry.from_shell_profiles(
            [(0.048, 0.105), (0.050, 0.305), (0.046, 0.340)],
            [(0.034, 0.112), (0.034, 0.335), (0.033, 0.340)],
            segments=64,
            start_cap="flat",
            end_cap="round",
            lip_samples=5,
        )
    )

    for i in range(5):
        yaw = i * 2.0 * math.pi / 5.0
        # Slightly rounded radial spokes overlap the center hub and the caster
        # crossbar, making the root link a supported single star-shaped frame.
        geom.merge(
            CapsuleGeometry(0.026, 0.230, radial_segments=20)
            .rotate_y(math.pi / 2.0)
            .rotate_z(yaw)
            .translate(0.196 * math.cos(yaw), 0.196 * math.sin(yaw), 0.093)
        )
        geom.merge(_local_box((0.055, 0.038, 0.040), (0.300, 0.0, 0.120), yaw))
        geom.merge(_local_box((0.075, 0.040, 0.026), (0.350, 0.0, 0.144), yaw))
        # Caster fork: two side cheeks bridged at the top.  Local X is radial
        # and local Y is tangential before the yaw rotation.
        for side in (-1.0, 1.0):
            geom.merge(_local_box((0.062, 0.010, 0.130), (0.392, side * 0.027, 0.078), yaw))
        geom.merge(_local_box((0.072, 0.068, 0.014), (0.383, 0.0, 0.146), yaw))
        geom.merge(_local_box((0.080, 0.060, 0.026), (0.350, 0.0, 0.133), yaw))

    return geom


def _seat_frame_geometry() -> MeshGeometry:
    geom = MeshGeometry()

    # Under-seat structural tray and the fixed part of the back frame.
    geom.merge(_rounded_slab((0.370, 0.380), 0.034, 0.040, (0.020, 0.0, 0.092)))
    geom.merge(_box((0.310, 0.080, 0.028), (-0.150, 0.0, 0.108)))
    geom.merge(_box((0.070, 0.410, 0.026), (-0.030, 0.0, 0.108)))

    for side in (-1.0, 1.0):
        geom.merge(_capsule_between((-0.195, side * 0.165, 0.105), (-0.255, side * 0.210, 0.250), 0.015))
        geom.merge(_capsule_between((-0.255, side * 0.220, 0.240), (-0.350, side * 0.215, 0.715), 0.017))

        # Flip-up arm clevis, mechanically tied back to the seat tray.
        geom.merge(_box((0.070, 0.030, 0.180), (-0.120, side * 0.252, 0.178)))
        for offset in (0.017, -0.017):
            geom.merge(_box((0.060, 0.006, 0.064), (-0.120, side * (0.290 + offset), 0.255)))
        geom.merge(_box((0.064, 0.064, 0.012), (-0.120, side * 0.290, 0.218)))

    geom.merge(_capsule_between((-0.255, -0.220, 0.245), (-0.255, 0.220, 0.245), 0.016))
    geom.merge(_capsule_between((-0.350, -0.215, 0.715), (-0.350, 0.215, 0.715), 0.017))

    return geom


def _back_mesh_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    z0 = 0.275
    z1 = 0.680
    x0 = -0.265
    x1 = -0.338

    def x_at(z: float) -> float:
        t = (z - z0) / (z1 - z0)
        return x0 + t * (x1 - x0)

    # Woven-looking vertical and horizontal strands, clipped inside the rigid
    # back frame and touching it at the side rails.
    for y in (-0.180, -0.120, -0.060, 0.0, 0.060, 0.120, 0.180):
        geom.merge(_capsule_between((x0, y, z0), (x1, y, z1), 0.0032))
    for z in (0.305, 0.355, 0.405, 0.455, 0.505, 0.555, 0.605, 0.655):
        geom.merge(_box((0.006, 0.410, 0.0045), (x_at(z), 0.0, z)))

    return geom


def _arm_frame_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    geom.merge(CylinderGeometry(0.018, 0.028, radial_segments=32).rotate_x(math.pi / 2.0))
    geom.merge(_capsule_between((0.0, 0.0, 0.0), (0.035, 0.0, 0.145), 0.013))
    geom.merge(_capsule_between((0.035, 0.0, 0.145), (0.280, 0.0, 0.145), 0.012))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_office_chair")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    mesh_fabric = model.material("charcoal_mesh", rgba=(0.02, 0.027, 0.030, 0.72))
    cushion_fabric = model.material("black_fabric", rgba=(0.035, 0.038, 0.042, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.07, 0.075, 0.080, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.008, 0.008, 0.009, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.32, 0.34, 0.35, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_base_frame_geometry(), "five_star_base"),
        material=dark_plastic,
        name="base_frame",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_geometry(_seat_frame_geometry(), "seat_frame"),
        material=dark_plastic,
        name="seat_frame",
    )
    seat.visual(
        mesh_from_geometry(_rounded_slab((0.500, 0.510), 0.070, 0.075, (0.045, 0.0, 0.138)), "seat_cushion"),
        material=cushion_fabric,
        name="cushion",
    )
    seat.visual(
        mesh_from_geometry(CylinderGeometry(0.027, 0.340, radial_segments=48).translate(0.0, 0.0, -0.060), "inner_mast"),
        material=chrome,
        name="inner_mast",
    )
    seat.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(0.0350, -0.174), (0.0350, -0.158)],
                [(0.0265, -0.172), (0.0265, -0.160)],
                segments=48,
                start_cap="round",
                end_cap="round",
                lip_samples=3,
            ),
            "guide_bushing",
        ),
        material=dark_plastic,
        name="guide_bushing",
    )
    seat.visual(
        mesh_from_geometry(_back_mesh_geometry(), "mesh_back"),
        material=mesh_fabric,
        name="back_mesh",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=0.0, upper=0.120),
    )

    tire = TireGeometry(
        0.052,
        0.032,
        inner_radius=0.031,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.003),
    )
    wheel = WheelGeometry(
        0.033,
        0.034,
        rim=WheelRim(inner_radius=0.020, flange_height=0.003, flange_thickness=0.002),
        hub=WheelHub(radius=0.014, width=0.026, cap_style="domed"),
        face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0022, window_radius=0.005),
        bore=WheelBore(style="round", diameter=0.007),
    )

    for i in range(5):
        yaw = i * 2.0 * math.pi / 5.0
        caster = model.part(f"caster_wheel_{i}")
        caster.visual(mesh_from_geometry(tire, f"caster_tire_{i}"), material=rubber, name="tire")
        caster.visual(mesh_from_geometry(wheel, f"caster_rim_{i}"), material=wheel_gray, name="rim")
        caster.visual(
            mesh_from_geometry(
                CylinderGeometry(0.006, 0.046, radial_segments=24).rotate_y(math.pi / 2.0),
                f"caster_axle_{i}",
            ),
            material=chrome,
            name="axle",
        )
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(
                xyz=(0.392 * math.cos(yaw), 0.392 * math.sin(yaw), 0.052),
                rpy=(0.0, 0.0, yaw + math.pi / 2.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    for name, side in (("left_armrest", 1.0), ("right_armrest", -1.0)):
        arm = model.part(name)
        arm.visual(
            mesh_from_geometry(_arm_frame_geometry(), f"{name}_frame"),
            material=dark_plastic,
            name="arm_frame",
        )
        arm.visual(
            mesh_from_geometry(_rounded_slab((0.350, 0.078), 0.034, 0.030, (0.195, 0.0, 0.165)), f"{name}_pad"),
            material=matte_black,
            name="pad",
        )
        model.articulation(
            f"seat_to_{name}",
            ArticulationType.REVOLUTE,
            parent=seat,
            child=arm,
            origin=Origin(xyz=(-0.120, side * 0.290, 0.255)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=0.0, upper=1.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    lift = object_model.get_articulation("base_to_seat")
    left_arm = object_model.get_part("left_armrest")
    right_arm = object_model.get_part("right_armrest")
    left_hinge = object_model.get_articulation("seat_to_left_armrest")
    right_hinge = object_model.get_articulation("seat_to_right_armrest")

    ctx.allow_overlap(
        base,
        seat,
        elem_a="base_frame",
        elem_b="guide_bushing",
        reason=(
            "The low-friction gas-lift guide bushing is intentionally captured "
            "with a slight hidden interference inside the hollow sleeve so the "
            "telescoping column is physically supported."
        ),
    )
    ctx.expect_within(
        seat,
        base,
        axes="xy",
        inner_elem="guide_bushing",
        outer_elem="base_frame",
        margin=0.002,
        name="guide bushing is captured by sleeve",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="z",
        elem_a="guide_bushing",
        elem_b="base_frame",
        min_overlap=0.012,
        name="guide bushing sits inside sleeve height",
    )

    # The gas-lift mast is a retained telescoping fit in the hollow sleeve.
    ctx.expect_within(
        seat,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="base_frame",
        margin=0.002,
        name="gas mast stays centered in sleeve",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="base_frame",
        min_overlap=0.090,
        name="gas mast retained at low height",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.120}):
        high_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            seat,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="base_frame",
            min_overlap=0.070,
            name="gas mast retained at high height",
        )
    ctx.check(
        "seat height slider raises chair",
        rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, high={high_pos}",
    )

    # Each arm has a hinge barrel nested between a supported side clevis, and at
    # the upper limit the pad flips well upward rather than sliding off the seat.
    for arm, joint, side_name in (
        (left_arm, left_hinge, "left"),
        (right_arm, right_hinge, "right"),
    ):
        ctx.expect_overlap(
            arm,
            seat,
            axes="xz",
            elem_a="arm_frame",
            elem_b="seat_frame",
            min_overlap=0.025,
            name=f"{side_name} arm hinge overlaps supported clevis in projection",
        )
        low_box = ctx.part_element_world_aabb(arm, elem="pad")
        with ctx.pose({joint: 1.70}):
            high_box = ctx.part_element_world_aabb(arm, elem="pad")
        low_top = low_box[1][2] if low_box is not None else None
        high_top = high_box[1][2] if high_box is not None else None
        ctx.check(
            f"{side_name} armrest flips upward",
            low_top is not None and high_top is not None and high_top > low_top + 0.15,
            details=f"low_top={low_top}, high_top={high_top}",
        )

    for i in range(5):
        caster = object_model.get_part(f"caster_wheel_{i}")
        ctx.allow_overlap(
            base,
            caster,
            elem_a="base_frame",
            elem_b="axle",
            reason="The caster axle is intentionally captured in the fork cheeks with a small hidden seated fit.",
        )
        ctx.expect_contact(
            base,
            caster,
            elem_a="base_frame",
            elem_b="axle",
            contact_tol=0.003,
            name=f"caster {i} axle is captured by fork",
        )
        wheel_joint = object_model.get_articulation(f"base_to_caster_{i}")
        ctx.check(
            f"caster {i} spins continuously",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={wheel_joint.articulation_type}",
        )

    return ctx.report()


object_model = build_object_model()
