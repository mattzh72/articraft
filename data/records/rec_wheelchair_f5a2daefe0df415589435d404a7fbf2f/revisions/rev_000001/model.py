from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
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
)


def _axis_to_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    ax, ay, az = axis
    length = math.sqrt(ax * ax + ay * ay + az * az)
    if length <= 1e-9:
        return (0.0, 0.0, 0.0)
    ax, ay, az = ax / length, ay / length, az / length
    yaw = math.atan2(ay, ax)
    pitch = math.atan2(math.sqrt(ax * ax + ay * ay), az)
    return (0.0, pitch, yaw)


def _tube_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=_axis_to_rpy((dx, dy, dz)),
        ),
        material=material,
        name=name,
    )


def _add_rear_wheel(part, index: int, *, outside_sign: float, materials: dict[str, object]) -> None:
    rim = mesh_from_geometry(
        WheelGeometry(
            0.270,
            0.038,
            rim=WheelRim(inner_radius=0.205, flange_height=0.010, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.038,
                width=0.044,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.046, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=16, thickness=0.0028, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.022),
        ),
        f"rear_wheel_rim_{index}",
    )
    tire = mesh_from_geometry(
        TireGeometry(
            0.315,
            0.048,
            inner_radius=0.274,
            tread=TireTread(style="circumferential", depth=0.003, count=4),
            grooves=(
                TireGroove(center_offset=-0.010, width=0.004, depth=0.0018),
                TireGroove(center_offset=0.010, width=0.004, depth=0.0018),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
        ),
        f"rear_tire_{index}",
    )
    push_rim_geom = TorusGeometry(0.285, 0.0055, radial_segments=18, tubular_segments=80).rotate_y(
        math.pi / 2.0
    )
    push_rim = mesh_from_geometry(push_rim_geom, f"push_rim_{index}")

    part.visual(rim, material=materials["polished_aluminum"], name="rim")
    part.visual(tire, material=materials["black_rubber"], name="tire")
    part.visual(
        push_rim,
        origin=Origin(xyz=(outside_sign * 0.037, 0.0, 0.0)),
        material=materials["brushed_steel"],
        name="hand_rim",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(-outside_sign * 0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_plastic"],
        name="inner_bearing_collar",
    )

    for spoke_index in range(8):
        angle = 2.0 * math.pi * spoke_index / 8.0
        y = math.cos(angle) * 0.285
        z = math.sin(angle) * 0.285
        _tube_between(
            part,
            (outside_sign * 0.013, y, z),
            (outside_sign * 0.037, y, z),
            radius=0.0026,
            material=materials["brushed_steel"],
            name=f"hand_rim_standoff_{spoke_index}",
        )


def _add_caster_wheel(part, index: int, *, materials: dict[str, object]) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.055,
                0.030,
                rim=WheelRim(inner_radius=0.037, flange_height=0.004, flange_thickness=0.002),
                hub=WheelHub(radius=0.018, width=0.032, cap_style="flat"),
                face=WheelFace(dish_depth=0.002, front_inset=0.001),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.0022, window_radius=0.004),
                bore=WheelBore(style="round", diameter=0.014),
            ),
            f"caster_rim_{index}",
        ),
        material=materials["polished_aluminum"],
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.085,
                0.036,
                inner_radius=0.057,
                tread=TireTread(style="circumferential", depth=0.002, count=2),
                sidewall=TireSidewall(style="rounded", bulge=0.035),
            ),
            f"caster_tire_{index}",
        ),
        material=materials["black_rubber"],
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_plastic"],
        name="hub_sleeve",
    )


def _add_caster_fork(part, *, materials: dict[str, object]) -> None:
    steel = materials["brushed_steel"]
    dark = materials["dark_plastic"]
    part.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="swivel_cap",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=steel,
        name="stem",
    )
    part.visual(
        Box((0.082, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.040, -0.050)),
        material=steel,
        name="fork_crown",
    )
    for side, x in enumerate((-0.030, 0.030)):
        part.visual(
            Box((0.010, 0.014, 0.126)),
            origin=Origin(xyz=(x, 0.043, -0.122)),
            material=steel,
            name=f"fork_blade_{side}",
        )
        _tube_between(
            part,
            (0.0, 0.0, -0.025),
            (x, 0.043, -0.080),
            radius=0.006,
            material=steel,
            name=f"fork_brace_{side}",
        )
    _tube_between(
        part,
        (-0.045, 0.045, -0.155),
        (0.045, 0.045, -0.155),
        radius=0.0055,
        material=dark,
        name="caster_axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    materials = {
        "brushed_steel": model.material("brushed_steel", rgba=(0.70, 0.72, 0.73, 1.0)),
        "polished_aluminum": model.material("polished_aluminum", rgba=(0.82, 0.84, 0.86, 1.0)),
        "black_rubber": model.material("black_rubber", rgba=(0.015, 0.015, 0.016, 1.0)),
        "dark_plastic": model.material("dark_plastic", rgba=(0.07, 0.075, 0.08, 1.0)),
        "blue_fabric": model.material("blue_sling_fabric", rgba=(0.05, 0.09, 0.18, 1.0)),
        "footplate": model.material("matte_footplate", rgba=(0.12, 0.12, 0.13, 1.0)),
    }

    frame = model.part("frame")
    steel = materials["brushed_steel"]
    fabric = materials["blue_fabric"]
    dark = materials["dark_plastic"]
    footplate = materials["footplate"]

    # Sling-style seat and backrest are thin and suspended inside a light open frame.
    frame.visual(
        Box((0.490, 0.430, 0.030)),
        origin=Origin(xyz=(0.0, 0.015, 0.500)),
        material=fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.530, 0.040, 0.480)),
        origin=Origin(xyz=(0.0, -0.222, 0.760)),
        material=fabric,
        name="back_sling",
    )
    frame.visual(
        Box((0.150, 0.040, 0.030)),
        origin=Origin(xyz=(-0.260, -0.380, 1.052)),
        material=dark,
        name="push_grip_0",
    )
    frame.visual(
        Box((0.150, 0.040, 0.030)),
        origin=Origin(xyz=(0.260, -0.380, 1.052)),
        material=dark,
        name="push_grip_1",
    )

    # Slender side frames and cross-members. Slight endpoint intersections represent welded tube joints.
    for side_index, x in enumerate((-0.260, 0.260)):
        rear_axle = (x, -0.185, 0.330)
        front_socket = (x, 0.345, 0.222)
        seat_front = (x, 0.235, 0.506)
        seat_rear = (x, -0.205, 0.506)
        back_mid = (x, -0.232, 0.800)
        back_top = (x, -0.255, 1.045)
        handle_end = (x, -0.390, 1.052)

        _tube_between(frame, rear_axle, front_socket, radius=0.0125, material=steel, name=f"lower_side_tube_{side_index}")
        _tube_between(frame, front_socket, seat_front, radius=0.0125, material=steel, name=f"front_upright_{side_index}")
        _tube_between(frame, seat_front, seat_rear, radius=0.0125, material=steel, name=f"seat_side_tube_{side_index}")
        _tube_between(frame, rear_axle, back_mid, radius=0.0125, material=steel, name=f"rear_upright_{side_index}")
        _tube_between(frame, back_mid, back_top, radius=0.0125, material=steel, name=f"back_post_{side_index}")
        _tube_between(frame, back_top, handle_end, radius=0.0125, material=steel, name=f"push_handle_tube_{side_index}")
        _tube_between(frame, front_socket, seat_rear, radius=0.0090, material=steel, name=f"diagonal_brace_{side_index}")
        _tube_between(frame, rear_axle, seat_front, radius=0.0090, material=steel, name=f"seat_brace_{side_index}")

        # Rear axle spindles and front caster sockets mark the actual joint hardware.
        outward = -1.0 if x < 0.0 else 1.0
        _tube_between(
            frame,
            (x, -0.185, 0.330),
            (x + outward * 0.067, -0.185, 0.330),
            radius=0.010,
            material=dark,
            name="rear_axle_stub_0" if side_index == 0 else "rear_axle_stub_1",
        )
        frame.visual(
            Cylinder(radius=0.021, length=0.038),
            origin=Origin(xyz=(x * 0.88, 0.380, 0.289)),
            material=steel,
            name=f"caster_socket_{side_index}",
        )
        _tube_between(
            frame,
            front_socket,
            (x * 0.88 + (-0.034 if x < 0.0 else 0.034), 0.380, 0.315),
            radius=0.0085,
            material=steel,
            name="caster_mount_tube_0" if side_index == 0 else "caster_mount_tube_1",
        )
        _tube_between(
            frame,
            (x * 0.88, 0.380, 0.303),
            (x * 0.88 + (-0.034 if x < 0.0 else 0.034), 0.380, 0.315),
            radius=0.0085,
            material=steel,
            name=f"caster_socket_lug_{side_index}",
        )

    _tube_between(frame, (-0.260, 0.235, 0.506), (0.260, 0.235, 0.506), radius=0.011, material=steel, name="front_seat_crossbar")
    _tube_between(frame, (-0.260, -0.205, 0.506), (0.260, -0.205, 0.506), radius=0.011, material=steel, name="rear_seat_crossbar")
    _tube_between(frame, (-0.260, -0.185, 0.330), (0.260, -0.185, 0.330), radius=0.012, material=steel, name="rear_axle_tube")
    _tube_between(frame, (-0.260, 0.345, 0.222), (0.260, 0.345, 0.222), radius=0.010, material=steel, name="front_lower_crossbar")
    _tube_between(frame, (-0.260, -0.255, 1.045), (0.260, -0.255, 1.045), radius=0.010, material=steel, name="back_top_crossbar")
    _tube_between(frame, (-0.260, -0.205, 0.506), (0.260, 0.235, 0.506), radius=0.008, material=steel, name="seat_x_brace_0")
    _tube_between(frame, (0.260, -0.205, 0.506), (-0.260, 0.235, 0.506), radius=0.008, material=steel, name="seat_x_brace_1")

    # Footrests: two small plates tied back to the front frame by skinny tubes.
    for side_index, x in enumerate((-0.130, 0.130)):
        frame.visual(
            Box((0.170, 0.115, 0.020)),
            origin=Origin(xyz=(x, 0.630, 0.135)),
            material=footplate,
            name=f"foot_plate_{side_index}",
        )
        _tube_between(frame, (x, 0.345, 0.222), (x, 0.590, 0.138), radius=0.008, material=steel, name=f"footrest_tube_{side_index}")
        _tube_between(frame, (x - 0.055 if x > 0 else x + 0.055, 0.590, 0.140), (x, 0.660, 0.140), radius=0.007, material=steel, name=f"footplate_edge_tube_{side_index}")

    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_1 = model.part("rear_wheel_1")
    _add_rear_wheel(rear_wheel_0, 0, outside_sign=-1.0, materials=materials)
    _add_rear_wheel(rear_wheel_1, 1, outside_sign=1.0, materials=materials)

    caster_fork_0 = model.part("caster_fork_0")
    caster_fork_1 = model.part("caster_fork_1")
    _add_caster_fork(caster_fork_0, materials=materials)
    _add_caster_fork(caster_fork_1, materials=materials)

    caster_wheel_0 = model.part("caster_wheel_0")
    caster_wheel_1 = model.part("caster_wheel_1")
    _add_caster_wheel(caster_wheel_0, 0, materials=materials)
    _add_caster_wheel(caster_wheel_1, 1, materials=materials)

    # Rear wheels spin continuously on side-mounted axles.
    model.articulation(
        "rear_wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.352, -0.185, 0.330)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=24.0),
    )
    model.articulation(
        "rear_wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(0.352, -0.185, 0.330)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=24.0),
    )

    # Each front caster has a free vertical swivel and a free wheel spin axle.
    for index, x in enumerate((-0.229, 0.229)):
        fork = caster_fork_0 if index == 0 else caster_fork_1
        wheel = caster_wheel_0 if index == 0 else caster_wheel_1
        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(x, 0.380, 0.271)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"caster_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.045, -0.155)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    caster_fork_0 = object_model.get_part("caster_fork_0")
    caster_fork_1 = object_model.get_part("caster_fork_1")
    caster_wheel_0 = object_model.get_part("caster_wheel_0")
    caster_wheel_1 = object_model.get_part("caster_wheel_1")

    ctx.check("manual_wheelchair_has_light_frame", frame is not None, "Expected a rooted tubular chair frame.")
    ctx.check(
        "wheelchair_primary_joint_count",
        len(object_model.articulations) == 6,
        f"Expected 2 rear spin, 2 caster swivel, and 2 caster spin joints; got {len(object_model.articulations)}.",
    )

    if frame and rear_wheel_0 and rear_wheel_1:
        ctx.allow_overlap(
            frame,
            rear_wheel_0,
            elem_a="rear_axle_stub_0",
            elem_b="inner_bearing_collar",
            reason="The fixed axle stub is intentionally captured inside the wheel bearing collar.",
        )
        ctx.allow_overlap(
            frame,
            rear_wheel_1,
            elem_a="rear_axle_stub_1",
            elem_b="inner_bearing_collar",
            reason="The fixed axle stub is intentionally captured inside the wheel bearing collar.",
        )
        ctx.expect_overlap(rear_wheel_0, frame, axes="z", min_overlap=0.25, name="rear wheel 0 reaches seat height")
        ctx.expect_overlap(rear_wheel_1, frame, axes="z", min_overlap=0.25, name="rear wheel 1 reaches seat height")
        ctx.expect_overlap(
            rear_wheel_0,
            frame,
            axes="xy",
            elem_a="inner_bearing_collar",
            min_overlap=0.010,
            name="rear wheel 0 bearing retained by axle",
        )
        ctx.expect_overlap(
            rear_wheel_1,
            frame,
            axes="xy",
            elem_a="inner_bearing_collar",
            min_overlap=0.010,
            name="rear wheel 1 bearing retained by axle",
        )

    swivel = object_model.get_articulation("caster_swivel_0")
    if caster_fork_0 and caster_wheel_0 and caster_fork_1 and caster_wheel_1:
        ctx.allow_overlap(
            caster_fork_0,
            caster_wheel_0,
            elem_a="caster_axle",
            elem_b="hub_sleeve",
            reason="The caster wheel hub sleeve is intentionally captured on the fork axle.",
        )
        ctx.allow_overlap(
            caster_fork_1,
            caster_wheel_1,
            elem_a="caster_axle",
            elem_b="hub_sleeve",
            reason="The caster wheel hub sleeve is intentionally captured on the fork axle.",
        )
        ctx.expect_overlap(
            caster_wheel_0,
            caster_fork_0,
            axes="xy",
            elem_a="hub_sleeve",
            min_overlap=0.014,
            name="caster wheel 0 hub captured on axle",
        )
        ctx.expect_overlap(
            caster_wheel_1,
            caster_fork_1,
            axes="xy",
            elem_a="hub_sleeve",
            min_overlap=0.014,
            name="caster wheel 1 hub captured on axle",
        )

    if caster_fork_0 and caster_wheel_0 and swivel:
        rest_pos = ctx.part_world_position(caster_wheel_0)
        with ctx.pose({swivel: math.pi / 2.0}):
            turned_pos = ctx.part_world_position(caster_wheel_0)
        ctx.check(
            "caster_swivel_moves_wheel_offset",
            rest_pos is not None
            and turned_pos is not None
            and abs(turned_pos[0] - rest_pos[0]) > 0.035
            and abs(turned_pos[1] - rest_pos[1]) > 0.035,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    return ctx.report()


object_model = build_object_model()
