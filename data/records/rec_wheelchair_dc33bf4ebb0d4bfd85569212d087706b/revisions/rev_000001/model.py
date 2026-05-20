from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_manual_wheelchair")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_fabric = model.material("navy_sling_fabric", rgba=(0.03, 0.055, 0.10, 1.0))
    seam = model.material("stitched_black_webbing", rgba=(0.01, 0.012, 0.015, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    grey_rubber = model.material("dark_grey_tire", rgba=(0.025, 0.025, 0.023, 1.0))
    spoke_metal = model.material("polished_spoke_metal", rgba=(0.86, 0.86, 0.82, 1.0))
    brake_red = model.material("red_brake_tips", rgba=(0.70, 0.02, 0.015, 1.0))

    def tube(
        part,
        name: str,
        points: list[tuple[float, float, float]],
        radius: float,
        material: Material,
        *,
        samples: int = 8,
        radial_segments: int = 18,
    ) -> None:
        try:
            geom = tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples,
                radial_segments=radial_segments,
                cap_ends=True,
            )
        except Exception as exc:  # pragma: no cover - diagnostic context for compile-time geometry errors
            raise RuntimeError(f"failed to build tube {name}: {points}") from exc
        part.visual(mesh_from_geometry(geom, name), material=material, name=name)

    def cyl_x(part, name: str, radius: float, length: float, center, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name: str, radius: float, length: float, center, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_z(part, name: str, radius: float, length: float, center, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center),
            material=material,
            name=name,
        )

    frame = model.part("frame")

    side_x = 0.255
    tube_r = 0.014
    for suffix, x in (("right", side_x), ("left", -side_x)):
        # One bent side rail gives the characteristic wheelchair silhouette:
        # rear axle, back post, push-handle bend, seat rail, and front caster leg.
        tube(
            frame,
            f"{suffix}_side_frame",
            [
                (x, -0.095, 0.315),
                (x, -0.205, 0.515),
                (x, -0.222, 0.830),
                (x, -0.170, 0.515),
                (x, 0.285, 0.515),
                (x, 0.345, 0.230),
                (x, 0.155, 0.255),
                (x, -0.090, 0.325),
            ],
            tube_r,
            aluminum,
            samples=7,
        )
        tube(
            frame,
            f"{suffix}_push_handle_tube",
            [
                (x, -0.222, 0.830),
                (x, -0.300, 0.905),
                (x, -0.370, 0.905),
                (x, -0.405, 0.905),
            ],
            tube_r,
            aluminum,
            samples=7,
        )
        tube(
            frame,
            f"{suffix}_diagonal_brace",
            [
                (x, -0.095, 0.315),
                (x, 0.035, 0.405),
                (x, 0.285, 0.515),
            ],
            0.010,
            aluminum,
            samples=6,
        )
        cyl_y(frame, f"{suffix}_push_grip", 0.021, 0.125, (x, -0.405, 0.905), rubber)
        cyl_z(frame, f"{suffix}_caster_socket", 0.024, 0.055, (x, 0.345, 0.202), aluminum)
        cyl_x(frame, f"{suffix}_rear_axle", 0.012, 0.098, (x + math.copysign(0.049, x), -0.065, 0.315), aluminum)
        cyl_z(frame, f"{suffix}_footrest_post", 0.012, 0.265, (x * 0.62, 0.355, 0.250), aluminum)
        tube(
            frame,
            f"{suffix}_footrest_strut",
            [
                (x, 0.285, 0.515),
                (x * 0.72, 0.330, 0.335),
                (x * 0.62, 0.355, 0.120),
            ],
            0.009,
            aluminum,
            samples=6,
        )
        tube(
            frame,
            f"{suffix}_brake_mount_strut",
            [
                (x, 0.020, 0.515),
                (x + math.copysign(0.010, x), 0.004, 0.480),
                (x + math.copysign(0.015, x), -0.004, 0.455),
            ],
            0.008,
            aluminum,
            samples=5,
        )
        frame.visual(
            Box((0.030, 0.026, 0.024)),
            origin=Origin(xyz=(x + math.copysign(0.005, x), -0.008, 0.455)),
            material=spoke_metal,
            name=f"{suffix}_brake_mount",
        )

    # Cross tubes and the folding-style X brace tie the two side frames together.
    cyl_x(frame, "front_seat_crossbar", 0.012, 0.535, (0.0, 0.285, 0.515), aluminum)
    cyl_x(frame, "rear_seat_crossbar", 0.012, 0.535, (0.0, -0.170, 0.515), aluminum)
    cyl_x(frame, "lower_front_crossbar", 0.011, 0.535, (0.0, 0.190, 0.255), aluminum)
    cyl_x(frame, "back_top_crossbar", 0.010, 0.515, (0.0, -0.225, 0.820), aluminum)
    tube(
        frame,
        "folding_x_brace_a",
        [(-0.255, -0.170, 0.515), (-0.040, 0.050, 0.405), (0.255, 0.285, 0.515)],
        0.008,
        aluminum,
        samples=5,
    )
    tube(
        frame,
        "folding_x_brace_b",
        [(0.255, -0.170, 0.515), (0.040, 0.050, 0.405), (-0.255, 0.285, 0.515)],
        0.008,
        aluminum,
        samples=5,
    )
    cyl_y(frame, "x_brace_center_pin", 0.012, 0.040, (0.0, 0.050, 0.405), spoke_metal)

    # Sling seat and back upholstery, with raised webbing strips and edge seams.
    frame.visual(
        Box((0.455, 0.430, 0.030)),
        origin=Origin(xyz=(0.0, 0.060, 0.520)),
        material=dark_fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.445, 0.026, 0.350)),
        origin=Origin(xyz=(0.0, -0.222, 0.675)),
        material=dark_fabric,
        name="back_sling",
    )
    for x in (-0.185, 0.185):
        frame.visual(
            Box((0.026, 0.425, 0.010)),
            origin=Origin(xyz=(x, 0.060, 0.539)),
            material=seam,
            name=f"seat_side_webbing_{'left' if x < 0 else 'right'}",
        )
    for y in (-0.135, 0.250):
        frame.visual(
            Box((0.440, 0.020, 0.011)),
            origin=Origin(xyz=(0.0, y, 0.539)),
            material=seam,
            name=f"seat_edge_webbing_{'rear' if y < 0 else 'front'}",
        )
    for z in (0.560, 0.675, 0.790):
        frame.visual(
            Box((0.430, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, -0.238, z)),
            material=seam,
            name=f"back_lumbar_band_{int(z * 1000)}",
        )
    frame.visual(
        Box((0.530, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.035, 0.545)),
        material=seam,
        name="soft_front_edge",
    )

    # Arm pads read as padded supports bolted to the side frames.
    for suffix, x in (("right", side_x), ("left", -side_x)):
        frame.visual(
            Box((0.060, 0.350, 0.035)),
            origin=Origin(xyz=(x, 0.050, 0.675)),
            material=seam,
            name=f"{suffix}_arm_pad",
        )
        cyl_z(frame, f"{suffix}_arm_front_post", 0.009, 0.160, (x, 0.215, 0.595), aluminum)
        cyl_z(frame, f"{suffix}_arm_rear_post", 0.009, 0.160, (x, -0.120, 0.595), aluminum)

    def rear_wheel(name: str, side_sign: float):
        wheel = model.part(name)
        wheel_geom = WheelGeometry(
            0.258,
            0.042,
            rim=WheelRim(
                inner_radius=0.218,
                flange_height=0.010,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.046,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.008, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=20, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.018),
        )
        tire_geom = TireGeometry(
            0.312,
            0.052,
            inner_radius=0.247,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.0035, count=28, land_ratio=0.62),
            grooves=(
                TireGroove(center_offset=-0.010, width=0.004, depth=0.002),
                TireGroove(center_offset=0.010, width=0.004, depth=0.002),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        )
        hand_rim = TorusGeometry(0.272, 0.007, radial_segments=20, tubular_segments=96)
        hand_rim.rotate_y(math.pi / 2.0).translate(side_sign * 0.040, 0.0, 0.0)

        mounts = None
        for i in range(8):
            a = 2.0 * math.pi * i / 8.0
            p_outer = (side_sign * 0.039, 0.272 * math.cos(a), 0.272 * math.sin(a))
            p_inner = (side_sign * 0.018, 0.245 * math.cos(a), 0.245 * math.sin(a))
            p_mid = (
                side_sign * 0.028,
                0.258 * math.cos(a),
                0.258 * math.sin(a),
            )
            try:
                geom = tube_from_spline_points([p_outer, p_mid, p_inner], radius=0.0038, samples_per_segment=3, radial_segments=10)
            except Exception as exc:  # pragma: no cover
                raise RuntimeError(f"failed to build hand rim mount {name} {i}") from exc
            mounts = geom if mounts is None else mounts.merge(geom)

        wheel.visual(mesh_from_geometry(tire_geom, f"{name}_tire"), material=grey_rubber, name="tire")
        wheel.visual(mesh_from_geometry(wheel_geom, f"{name}_wheel_structure"), material=spoke_metal, name="wheel_structure")
        wheel.visual(mesh_from_geometry(hand_rim, f"{name}_hand_rim"), material=spoke_metal, name="hand_rim")
        if mounts is not None:
            wheel.visual(mesh_from_geometry(mounts, f"{name}_hand_rim_mounts"), material=spoke_metal, name="hand_rim_mounts")
        return wheel

    right_rear_wheel = rear_wheel("right_rear_wheel", 1.0)
    left_rear_wheel = rear_wheel("left_rear_wheel", -1.0)

    model.articulation(
        "right_rear_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(0.365, -0.065, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "left_rear_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.365, -0.065, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )

    def caster_yoke(name: str):
        yoke = model.part(name)
        cyl_z(yoke, "swivel_stem", 0.010, 0.070, (0.0, 0.0, -0.035), aluminum)
        yoke.visual(Box((0.078, 0.028, 0.018)), origin=Origin(xyz=(0.0, 0.0, -0.070)), material=aluminum, name="fork_crown")
        yoke.visual(Box((0.009, 0.028, 0.126)), origin=Origin(xyz=(0.029, 0.0, -0.130)), material=aluminum, name="fork_side_outer")
        yoke.visual(Box((0.009, 0.028, 0.126)), origin=Origin(xyz=(-0.029, 0.0, -0.130)), material=aluminum, name="fork_side_inner")
        cyl_x(yoke, "caster_axle", 0.0055, 0.070, (0.0, 0.0, -0.170), spoke_metal)
        return yoke

    def caster_wheel(name: str):
        wheel = model.part(name)
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.075,
                    0.034,
                    inner_radius=0.052,
                    tread=TireTread(style="smooth", depth=0.001, count=12, land_ratio=0.8),
                    sidewall=TireSidewall(style="rounded", bulge=0.03),
                    shoulder=TireShoulder(width=0.003, radius=0.002),
                ),
                f"{name}_tire",
            ),
            material=grey_rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.055,
                    0.030,
                    rim=WheelRim(inner_radius=0.035, flange_height=0.004, flange_thickness=0.002),
                    hub=WheelHub(radius=0.018, width=0.026, cap_style="flat"),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.006),
                    bore=WheelBore(style="round", diameter=0.010),
                ),
                f"{name}_wheel_structure",
            ),
            material=spoke_metal,
            name="wheel_structure",
        )
        return wheel

    right_caster = caster_yoke("right_caster")
    left_caster = caster_yoke("left_caster")
    right_caster_wheel = caster_wheel("right_caster_wheel")
    left_caster_wheel = caster_wheel("left_caster_wheel")

    for suffix, x, yoke, wheel in (
        ("right", side_x, right_caster, right_caster_wheel),
        ("left", -side_x, left_caster, left_caster_wheel),
    ):
        model.articulation(
            f"{suffix}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=yoke,
            origin=Origin(xyz=(x, 0.345, 0.205)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )
        model.articulation(
            f"{suffix}_caster_axle",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.170)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )

    # Folding footplates: at q=0 they are down and horizontal; positive q folds
    # the front edge upward for transfers.
    for suffix, x in (("right", side_x * 0.62), ("left", -side_x * 0.62)):
        plate = model.part(f"{suffix}_footplate")
        plate.visual(
            Box((0.135, 0.165, 0.014)),
            origin=Origin(xyz=(0.0, 0.085, -0.004)),
            material=seam,
            name="footplate_panel",
        )
        plate.visual(
            Box((0.125, 0.145, 0.006)),
            origin=Origin(xyz=(0.0, 0.087, 0.005)),
            material=rubber,
            name="ribbed_tread_pad",
        )
        cyl_x(plate, "footplate_hinge_barrel", 0.010, 0.150, (0.0, 0.0, 0.0), aluminum)
        model.articulation(
            f"{suffix}_footplate_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=plate,
            origin=Origin(xyz=(x, 0.376, 0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
        )

    # Wheel-lock levers are user controls and articulate as separate parts.
    for suffix, side in (("right", 1.0), ("left", -1.0)):
        lever = model.part(f"{suffix}_brake_lever")
        tube(
            lever,
            "lever_arm",
            [(side * 0.018, 0.0, 0.0), (side * 0.030, 0.045, 0.025), (side * 0.034, 0.115, 0.060)],
            0.006,
            aluminum,
            samples=5,
        )
        cyl_y(lever, "rubber_handle", 0.012, 0.055, (side * 0.038, 0.132, 0.068), brake_red)
        lever.visual(Box((0.038, 0.020, 0.018)), origin=Origin(xyz=(-side * 0.005, -0.005, 0.000)), material=spoke_metal, name="pivot_block")
        model.articulation(
            f"{suffix}_brake_pivot",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=lever,
            origin=Origin(xyz=(side * 0.270, 0.000, 0.455)),
            axis=(side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-0.35, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Captured rotating shafts and caster stems are intentionally nested in
    # simplified hub/socket proxy geometry.  Scope each allowance to the exact
    # named shaft/socket element and prove the retained insertion.
    for side in ("right", "left"):
        ctx.allow_overlap(
            "frame",
            f"{side}_rear_wheel",
            elem_a=f"{side}_rear_axle",
            elem_b="wheel_structure",
            reason="The fixed rear axle stub is intentionally captured inside the wheel hub.",
        )
        ctx.expect_overlap(
            "frame",
            f"{side}_rear_wheel",
            elem_a=f"{side}_rear_axle",
            elem_b="wheel_structure",
            axes="yz",
            min_overlap=0.020,
            name=f"{side} rear axle passes through hub center",
        )
        ctx.allow_overlap(
            "frame",
            f"{side}_caster",
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            reason="The caster swivel stem is seated inside the tubular frame socket.",
        )
        ctx.expect_overlap(
            "frame",
            f"{side}_caster",
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            axes="z",
            min_overlap=0.020,
            name=f"{side} caster stem remains inserted in socket",
        )
        ctx.allow_overlap(
            f"{side}_caster",
            f"{side}_caster_wheel",
            elem_a="caster_axle",
            elem_b="wheel_structure",
            reason="The small caster axle is intentionally captured in the wheel hub.",
        )
        ctx.expect_overlap(
            f"{side}_caster",
            f"{side}_caster_wheel",
            elem_a="caster_axle",
            elem_b="wheel_structure",
            axes="x",
            min_overlap=0.020,
            name=f"{side} caster axle spans wheel hub",
        )
        ctx.allow_overlap(
            "frame",
            f"{side}_footplate",
            elem_a=f"{side}_footrest_post",
            elem_b="footplate_hinge_barrel",
            reason="The footplate hinge barrel wraps around the front support post.",
        )
        ctx.expect_overlap(
            "frame",
            f"{side}_footplate",
            elem_a=f"{side}_footrest_post",
            elem_b="footplate_hinge_barrel",
            axes="xz",
            min_overlap=0.006,
            name=f"{side} footplate hinge is carried by post",
        )
        ctx.allow_overlap(
            "frame",
            f"{side}_brake_lever",
            elem_a=f"{side}_brake_mount",
            elem_b="pivot_block",
            reason="The wheel-lock lever pivot block is bolted around the side frame tube.",
        )
        ctx.expect_overlap(
            "frame",
            f"{side}_brake_lever",
            elem_a=f"{side}_brake_mount",
            elem_b="pivot_block",
            axes="xz",
            min_overlap=0.010,
            name=f"{side} brake lever pivot is mounted to side frame",
        )
        ctx.allow_overlap(
            "frame",
            f"{side}_brake_lever",
            elem_a=f"{side}_brake_mount_strut",
            elem_b="pivot_block",
            reason="The welded brake support strut disappears into the clamp block around the pivot.",
        )
        ctx.expect_overlap(
            "frame",
            f"{side}_brake_lever",
            elem_a=f"{side}_brake_mount_strut",
            elem_b="pivot_block",
            axes="xz",
            min_overlap=0.008,
            name=f"{side} brake support enters pivot clamp",
        )

    right_footplate = object_model.get_part("right_footplate")
    right_footplate_hinge = object_model.get_articulation("right_footplate_hinge")
    rest_aabb = ctx.part_world_aabb(right_footplate)
    with ctx.pose({right_footplate_hinge: 1.20}):
        folded_aabb = ctx.part_world_aabb(right_footplate)
    ctx.check(
        "footplate folds upward",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] > rest_aabb[1][2] + 0.09,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    right_caster = object_model.get_part("right_caster")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    rest_caster_aabb = ctx.part_world_aabb(right_caster)
    with ctx.pose({right_caster_swivel: math.pi / 2.0}):
        turned_caster_aabb = ctx.part_world_aabb(right_caster)
    ctx.check(
        "front caster swivels",
        rest_caster_aabb is not None
        and turned_caster_aabb is not None
        and abs((turned_caster_aabb[1][0] - turned_caster_aabb[0][0]) - (rest_caster_aabb[1][0] - rest_caster_aabb[0][0])) > 0.005,
        details=f"rest={rest_caster_aabb}, turned={turned_caster_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
