from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _permute_xyz_to_x_y_z_from_z_x_y(geometry):
    """Map an extrusion's local (profile_x, profile_y, depth_z) to (Y, Z, X)."""
    geometry.vertices = [(z, x, y) for (x, y, z) in geometry.vertices]
    return geometry


def _hollow_tube_mesh(name: str, *, inner_radius: float, outer_radius: float, z0: float, z1: float):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _rounded_pad_mesh(name: str, width_x: float, width_y: float, thickness_z: float, *, radius: float):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width_x, width_y, radius, corner_segments=10),
            thickness_z,
            center=True,
        ),
    )


def _back_pad_mesh(name: str, width_y: float, height_z: float, thickness_x: float):
    return _save_mesh(
        name,
        _permute_xyz_to_x_y_z_from_z_x_y(
            ExtrudeGeometry(
                rounded_rect_profile(width_y, height_z, 0.055, corner_segments=12),
                thickness_x,
                center=True,
            )
        ),
    )


def _make_wheel_meshes(prefix: str):
    return _save_mesh(
        f"{prefix}_tire",
        TireGeometry(
            0.055,
            0.032,
            inner_radius=0.036,
            tread=TireTread(style="block", depth=0.0025, count=18, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_office_chair")

    black_plastic = model.material("black_plastic", rgba=(0.035, 0.036, 0.038, 1.0))
    charcoal_fabric = model.material("charcoal_fabric", rgba=(0.10, 0.105, 0.115, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.73, 0.70, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.018, 0.018, 0.017, 1.0))
    light_rim = model.material("light_rim", rgba=(0.58, 0.60, 0.62, 1.0))

    lower_socket_mesh = _hollow_tube_mesh(
        "lower_gas_socket", inner_radius=0.037, outer_radius=0.062, z0=0.135, z1=0.280
    )
    star_hub_mesh = _hollow_tube_mesh(
        "star_hub_shell", inner_radius=0.058, outer_radius=0.155, z0=0.140, z1=0.210
    )
    socket_cap_mesh = _hollow_tube_mesh(
        "base_socket_cap", inner_radius=0.040, outer_radius=0.078, z0=0.280, z1=0.310
    )
    upper_socket_mesh = _hollow_tube_mesh(
        "upper_gas_socket", inner_radius=0.036, outer_radius=0.058, z0=-0.100, z1=0.025
    )
    seat_cushion_mesh = _rounded_pad_mesh("rounded_seat_cushion", 0.53, 0.49, 0.080, radius=0.080)
    arm_pad_mesh = _rounded_pad_mesh("soft_arm_pad", 0.36, 0.070, 0.035, radius=0.030)
    back_pad_mesh = _back_pad_mesh("low_back_pad", 0.50, 0.330, 0.060)
    base = model.part("star_base")
    base.visual(
        star_hub_mesh,
        material=dark_frame,
        name="central_hub",
    )
    base.visual(lower_socket_mesh, material=satin_metal, name="lower_socket")
    base.visual(
        socket_cap_mesh,
        material=dark_frame,
        name="socket_cap",
    )
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        c = cos(angle)
        s = sin(angle)
        base.visual(
            Box((0.380, 0.072, 0.046)),
            origin=Origin(xyz=(0.335 * c, 0.335 * s, 0.180), rpy=(0.0, 0.0, angle)),
            material=dark_frame,
            name=f"spoke_{i}",
        )
        base.visual(
            Cylinder(radius=0.052, length=0.040),
            origin=Origin(xyz=(0.505 * c, 0.505 * s, 0.185)),
            material=dark_frame,
            name=f"caster_pad_{i}",
        )

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=0.030, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=chrome,
        name="lift_column",
    )
    gas_column.visual(
        Cylinder(radius=0.041, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=satin_metal,
        name="lower_clip_ring",
    )
    gas_column.visual(
        Cylinder(radius=0.038, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=satin_metal,
        name="upper_clip_ring",
    )

    seat_support = model.part("seat_support")
    seat_support.visual(upper_socket_mesh, material=satin_metal, name="upper_socket")
    seat_support.visual(
        Cylinder(radius=0.085, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_frame,
        name="swivel_bearing_plate",
    )
    seat_support.visual(
        Box((0.430, 0.360, 0.040)),
        origin=Origin(xyz=(0.005, 0.0, 0.045)),
        material=dark_frame,
        name="underseat_plate",
    )
    seat_support.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.020, 0.0, 0.105)),
        material=charcoal_fabric,
        name="seat_cushion",
    )
    # Fixed arms: two bolted side frames tied into the under-seat plate.
    for side, y in (("left", 0.302), ("right", -0.302)):
        seat_support.visual(
            Box((0.030, 0.036, 0.230)),
            origin=Origin(xyz=(-0.165, y, 0.170)),
            material=dark_frame,
            name=f"{side}_rear_post",
        )
        seat_support.visual(
            Box((0.030, 0.036, 0.220)),
            origin=Origin(xyz=(0.150, y, 0.160)),
            material=dark_frame,
            name=f"{side}_front_post",
        )
        seat_support.visual(
            Box((0.340, 0.035, 0.030)),
            origin=Origin(xyz=(-0.005, y, 0.273)),
            material=dark_frame,
            name=f"{side}_arm_rail",
        )
        seat_support.visual(
            arm_pad_mesh,
            origin=Origin(xyz=(-0.005, y, 0.292)),
            material=black_plastic,
            name=f"{side}_arm_pad",
        )
        seat_support.visual(
            Box((0.060, 0.060, 0.030)),
            origin=Origin(xyz=(-0.165, y * 0.93, 0.060)),
            material=dark_frame,
            name=f"{side}_rear_foot",
        )
        seat_support.visual(
            Box((0.060, 0.060, 0.030)),
            origin=Origin(xyz=(0.150, y * 0.93, 0.060)),
            material=dark_frame,
            name=f"{side}_front_foot",
        )
        seat_support.visual(
            Box((0.070, 0.210, 0.034)),
            origin=Origin(xyz=(-0.165, y * 0.78, 0.062)),
            material=dark_frame,
            name=f"{side}_rear_mount",
        )
        seat_support.visual(
            Box((0.070, 0.210, 0.034)),
            origin=Origin(xyz=(0.150, y * 0.78, 0.062)),
            material=dark_frame,
            name=f"{side}_front_mount",
        )
    # Rear tilt yoke mounted to the under-seat plate.
    seat_support.visual(
        Box((0.160, 0.070, 0.035)),
        origin=Origin(xyz=(-0.245, 0.0, 0.100)),
        material=dark_frame,
        name="tilt_yoke_bridge",
    )
    for y in (-0.205, 0.205):
        seat_support.visual(
            Box((0.064, 0.030, 0.110)),
            origin=Origin(xyz=(-0.265, y, 0.155)),
            material=dark_frame,
            name=f"tilt_cheek_{'pos' if y > 0 else 'neg'}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.014, length=0.380),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_barrel",
    )
    backrest.visual(
        Box((0.075, 0.360, 0.026)),
        origin=Origin(xyz=(-0.035, 0.0, 0.010)),
        material=dark_frame,
        name="barrel_link",
    )
    backrest.visual(
        Box((0.030, 0.045, 0.230)),
        origin=Origin(xyz=(-0.070, 0.155, 0.130), rpy=(0.0, -0.20, 0.0)),
        material=dark_frame,
        name="left_back_strut",
    )
    backrest.visual(
        Box((0.030, 0.045, 0.230)),
        origin=Origin(xyz=(-0.070, -0.155, 0.130), rpy=(0.0, -0.20, 0.0)),
        material=dark_frame,
        name="right_back_strut",
    )
    backrest.visual(
        back_pad_mesh,
        origin=Origin(xyz=(-0.100, 0.0, 0.260), rpy=(0.0, -0.16, 0.0)),
        material=charcoal_fabric,
        name="back_pad",
    )
    backrest.visual(
        Cylinder(radius=0.012, length=0.455),
        origin=Origin(xyz=(-0.130, 0.0, 0.430), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="top_piping",
    )

    caster_specs: list[tuple[str, float]] = []
    for i in range(5):
        angle = 2.0 * pi * i / 5.0
        caster_specs.append((f"caster_{i}", angle))
    for name, angle in caster_specs:
        fork = model.part(f"{name}_fork")
        fork.visual(
            Cylinder(radius=0.014, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=satin_metal,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.044)),
            material=dark_frame,
            name="stem_collar",
        )
        fork.visual(
            Box((0.090, 0.038, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=dark_frame,
            name="fork_bridge",
        )
        for side, x in (("pos", 0.031), ("neg", -0.031)):
            fork.visual(
                Box((0.010, 0.034, 0.110)),
                origin=Origin(xyz=(x, 0.0, -0.100)),
                material=dark_frame,
                name=f"fork_cheek_{side}",
            )
            fork.visual(
                Cylinder(radius=0.0055, length=0.015),
                origin=Origin(xyz=(x, 0.0, -0.120), rpy=(0.0, pi / 2.0, 0.0)),
                material=satin_metal,
                name=f"axle_pin_{side}",
            )

        wheel = model.part(f"{name}_wheel")
        tire_mesh = _make_wheel_meshes(name)
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.039, length=0.030),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=light_rim,
            name="rim_disc",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.044),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_metal,
            name="hub_boss",
        )

        c = cos(angle)
        s = sin(angle)
        pivot_xyz = (0.505 * c, 0.505 * s, 0.175)
        model.articulation(
            f"{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=pivot_xyz, rpy=(0.0, 0.0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=8.0),
        )
        model.articulation(
            f"{name}_roll",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=25.0),
        )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gas_column,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.25, lower=0.0, upper=0.110),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat_support,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "back_tilt",
        ArticulationType.REVOLUTE,
        parent=seat_support,
        child=backrest,
        origin=Origin(xyz=(-0.265, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.10, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("star_base")
    gas = object_model.get_part("gas_column")
    seat = object_model.get_part("seat_support")
    back = object_model.get_part("backrest")
    height = object_model.get_articulation("height_slide")
    tilt = object_model.get_articulation("back_tilt")

    ctx.allow_overlap(
        gas,
        seat,
        elem_a="upper_clip_ring",
        elem_b="upper_socket",
        reason="The upper retaining collar is intentionally clipped inside the seat-support socket so the pedestal stays captured during swivel and height adjustment.",
    )
    ctx.allow_overlap(
        gas,
        base,
        elem_a="lower_clip_ring",
        elem_b="lower_socket",
        reason="The lower retaining collar is intentionally clipped inside the base socket to keep the pedestal captured in the five-star base.",
    )

    ctx.expect_within(
        gas,
        base,
        axes="xy",
        inner_elem="lift_column",
        outer_elem="lower_socket",
        margin=0.004,
        name="gas column is centered in the lower base socket",
    )
    ctx.expect_overlap(
        gas,
        base,
        axes="z",
        elem_a="lift_column",
        elem_b="lower_socket",
        min_overlap=0.10,
        name="gas column remains inserted in the base socket",
    )
    ctx.expect_within(
        gas,
        seat,
        axes="xy",
        inner_elem="lift_column",
        outer_elem="upper_socket",
        margin=0.004,
        name="gas column is centered in the seat support socket",
    )
    ctx.expect_overlap(
        gas,
        seat,
        axes="z",
        elem_a="lift_column",
        elem_b="upper_socket",
        min_overlap=0.09,
        name="seat support socket captures the gas column",
    )

    low_seat_pos = ctx.part_world_position(seat)
    with ctx.pose({height: 0.110}):
        high_seat_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            gas,
            base,
            axes="z",
            elem_a="lift_column",
            elem_b="lower_socket",
            min_overlap=0.10,
            name="raised gas column still clips into the base",
        )
        ctx.expect_overlap(
            gas,
            seat,
            axes="z",
            elem_a="lift_column",
            elem_b="upper_socket",
            min_overlap=0.09,
            name="raised gas column still clips into the seat support",
        )
    ctx.check(
        "height slide raises the supported seat",
        low_seat_pos is not None and high_seat_pos is not None and high_seat_pos[2] > low_seat_pos[2] + 0.09,
        details=f"low={low_seat_pos}, high={high_seat_pos}",
    )

    rest_back_aabb = ctx.part_element_world_aabb(back, elem="back_pad")
    with ctx.pose({tilt: 0.38}):
        tilted_back_aabb = ctx.part_element_world_aabb(back, elem="back_pad")
    rest_back_x = None if rest_back_aabb is None else (rest_back_aabb[0][0] + rest_back_aabb[1][0]) * 0.5
    tilted_back_x = None if tilted_back_aabb is None else (tilted_back_aabb[0][0] + tilted_back_aabb[1][0]) * 0.5
    ctx.check(
        "backrest tilt reclines rearward",
        rest_back_x is not None and tilted_back_x is not None and tilted_back_x < rest_back_x - 0.03,
        details=f"rest_back_x={rest_back_x}, tilted_back_x={tilted_back_x}",
    )

    for i in range(5):
        fork = object_model.get_part(f"caster_{i}_fork")
        wheel = object_model.get_part(f"caster_{i}_wheel")
        ctx.expect_contact(
            wheel,
            fork,
            contact_tol=0.010,
            name=f"caster {i} wheel is captured in its fork",
        )
        ctx.expect_gap(
            base,
            wheel,
            axis="z",
            min_gap=0.010,
            max_gap=0.120,
            name=f"caster {i} wheel is below the star base arm",
        )

    return ctx.report()


object_model = build_object_model()
