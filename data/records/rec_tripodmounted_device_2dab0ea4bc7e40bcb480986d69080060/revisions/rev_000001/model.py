from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return a visual origin that aligns a local-Z cylinder between two points."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
) -> None:
    origin, length = _cylinder_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripod_mounted_device")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.02, 0.026, 0.032, 0.88))
    soft_accent = model.material("muted_champagne", rgba=(0.72, 0.61, 0.43, 1.0))

    tripod = model.part("tripod")

    # Central column and rigid top plate.  The alternating satin/matte bands are
    # deliberately thin so the interfaces read as real seams rather than gaps.
    tripod.visual(
        Cylinder(radius=0.026, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=satin_graphite,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=matte_black,
        name="spreader_collar",
    )
    tripod.visual(
        Cylinder(radius=0.054, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=matte_black,
        name="leg_hub",
    )
    tripod.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.437)),
        material=brushed_aluminum,
        name="hub_trim_ring",
    )
    tripod.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.762)),
        material=satin_graphite,
        name="top_stem",
    )
    tripod.visual(
        Box(size=(0.220, 0.160, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.792)),
        material=matte_black,
        name="head_plate",
    )
    tripod.visual(
        Box(size=(0.210, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.079, 0.805)),
        material=brushed_aluminum,
        name="front_plate_seam",
    )
    tripod.visual(
        Box(size=(0.210, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.079, 0.805)),
        material=brushed_aluminum,
        name="rear_plate_seam",
    )
    tripod.visual(
        Cylinder(radius=0.057, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.803)),
        material=brushed_aluminum,
        name="pan_thrust_washer",
    )

    clevis_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (0.074, 0.054, 0.058),
            gap_width=0.028,
            bore_diameter=0.014,
            bore_center_z=0.032,
            base_thickness=0.013,
            corner_radius=0.004,
        ),
        "leg_clevis",
    )

    leg_hinges: list[object] = []
    for i in range(3):
        theta = math.radians(90.0 + i * 120.0)
        radial = (math.cos(theta), math.sin(theta), 0.0)
        tangent = (-math.sin(theta), math.cos(theta), 0.0)
        hinge = (0.068 * radial[0], 0.068 * radial[1], 0.420)
        yaw = theta + math.pi * 0.5

        tripod.visual(
            clevis_mesh,
            origin=Origin(xyz=hinge, rpy=(0.0, 0.0, yaw)),
            material=matte_black,
            name=f"leg_clevis_{i}",
        )

        # Rigid spreader links from the sliding collar to each leg.  They are
        # modeled as root-mounted links with visible end pivots so the support
        # logic is obvious without pretending to solve a closed-loop linkage.
        link_start = (
            0.026 * radial[0] + 0.018 * tangent[0],
            0.026 * radial[1] + 0.018 * tangent[1],
            0.295,
        )
        link_end = (
            0.285 * radial[0] + 0.055 * tangent[0],
            0.285 * radial[1] + 0.055 * tangent[1],
            0.210,
        )
        _add_cylinder_between(
            tripod,
            link_start,
            link_end,
            0.0065,
            material=brushed_aluminum,
            name=f"spreader_link_{i}",
        )
        tripod.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=link_start),
            material=soft_accent,
            name=f"inner_spreader_pivot_{i}",
        )
        tripod.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=link_end),
            material=soft_accent,
            name=f"outer_spreader_pivot_{i}",
        )

        leg = model.part(f"leg_{i}")
        knee = (0.305 * radial[0] - hinge[0], 0.305 * radial[1] - hinge[1], 0.205 - hinge[2])
        foot = (0.525 * radial[0] - hinge[0], 0.525 * radial[1] - hinge[1], 0.030 - hinge[2])
        near = (0.058 * radial[0], 0.058 * radial[1], -0.020)

        leg.visual(
            Cylinder(radius=0.013, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, yaw)),
            material=brushed_aluminum,
            name="hinge_barrel",
        )
        leg.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=near),
            material=satin_graphite,
            name="hinge_eye",
        )
        _add_cylinder_between(
            leg,
            (0.0, 0.0, 0.0),
            near,
            0.0075,
            material=satin_graphite,
            name="hinge_tongue",
        )
        _add_cylinder_between(
            leg,
            near,
            knee,
            0.014,
            material=satin_graphite,
            name="upper_tube",
        )
        _add_cylinder_between(
            leg,
            (0.265 * radial[0] - hinge[0], 0.265 * radial[1] - hinge[1], 0.230 - hinge[2]),
            foot,
            0.010,
            material=brushed_aluminum,
            name="lower_tube",
        )
        collar_point = (
            0.308 * radial[0] - hinge[0],
            0.308 * radial[1] - hinge[1],
            0.205 - hinge[2],
        )
        leg.visual(
            Cylinder(radius=0.018, length=0.058),
            origin=Origin(xyz=collar_point, rpy=(0.0, math.atan2(math.hypot(knee[0], knee[1]), -knee[2]), theta)),
            material=matte_black,
            name="twist_lock_collar",
        )
        leg.visual(
            Box(size=(0.010, 0.040, 0.020)),
            origin=Origin(
                xyz=(
                    collar_point[0] + 0.016 * tangent[0],
                    collar_point[1] + 0.016 * tangent[1],
                    collar_point[2],
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=soft_accent,
            name="lock_index_tab",
        )
        leg.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=foot),
            material=dark_rubber,
            name="foot_ball",
        )
        leg.visual(
            Box(size=(0.125, 0.055, 0.018)),
            origin=Origin(
                xyz=(
                    0.548 * radial[0] - hinge[0],
                    0.548 * radial[1] - hinge[1],
                    0.010 - hinge[2],
                ),
                rpy=(0.0, 0.0, theta),
            ),
            material=dark_rubber,
            name="foot_pad",
        )

        leg_hinges.append(
            model.articulation(
                f"tripod_to_leg_{i}",
                ArticulationType.REVOLUTE,
                parent=tripod,
                child=leg,
                origin=Origin(xyz=hinge),
                axis=tangent,
                motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.18, upper=0.32),
            )
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.054, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_graphite,
        name="pan_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=brushed_aluminum,
        name="pan_seam_ring",
    )
    pan_head.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.170, 0.064, 0.124),
                span_width=0.104,
                trunnion_diameter=0.024,
                trunnion_center_z=0.078,
                base_thickness=0.018,
                corner_radius=0.005,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=matte_black,
        name="tilt_yoke",
    )
    pan_head.visual(
        Box(size=(0.130, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.034, 0.070)),
        material=brushed_aluminum,
        name="rear_yoke_seam",
    )
    pan_joint = model.articulation(
        "tripod_to_pan",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.806)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brushed_aluminum,
        name="tilt_axle",
    )
    tilt_cradle.visual(
        Box(size=(0.064, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, 0.030, 0.004)),
        material=satin_graphite,
        name="axle_bridge",
    )
    tilt_cradle.visual(
        Box(size=(0.188, 0.018, 0.255)),
        origin=Origin(xyz=(0.0, 0.052, 0.060)),
        material=matte_black,
        name="bracket_backplate",
    )
    tilt_cradle.visual(
        Box(size=(0.012, 0.012, 0.242)),
        origin=Origin(xyz=(-0.082, 0.048, 0.060)),
        material=satin_graphite,
        name="guide_rail_0",
    )
    tilt_cradle.visual(
        Box(size=(0.012, 0.012, 0.242)),
        origin=Origin(xyz=(0.082, 0.048, 0.060)),
        material=satin_graphite,
        name="guide_rail_1",
    )
    tilt_cradle.visual(
        Box(size=(0.170, 0.052, 0.025)),
        origin=Origin(xyz=(0.0, 0.083, -0.080)),
        material=matte_black,
        name="lower_jaw",
    )
    tilt_cradle.visual(
        Box(size=(0.132, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, 0.097, -0.064)),
        material=dark_rubber,
        name="lower_jaw_pad",
    )
    tilt_cradle.visual(
        Box(size=(0.080, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.061, 0.184)),
        material=soft_accent,
        name="scale_marker",
    )
    tilt_joint = model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=-0.55, upper=0.75),
    )

    device = model.part("device")
    device.visual(
        Box(size=(0.132, 0.012, 0.188)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_graphite,
        name="device_body",
    )
    device.visual(
        Box(size=(0.119, 0.003, 0.164)),
        origin=Origin(xyz=(0.0, 0.0076, 0.004)),
        material=smoked_glass,
        name="front_glass",
    )
    device.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(-0.042, -0.0078, 0.062), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=smoked_glass,
        name="sensor_lens",
    )
    device.visual(
        Box(size=(0.034, 0.004, 0.010)),
        origin=Origin(xyz=(0.044, 0.0079, -0.074)),
        material=soft_accent,
        name="status_window",
    )
    model.articulation(
        "tilt_to_device",
        ArticulationType.FIXED,
        parent=tilt_cradle,
        child=device,
        origin=Origin(xyz=(0.0, 0.096, 0.035)),
    )

    top_jaw = model.part("top_jaw")
    top_jaw.visual(
        Box(size=(0.172, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="jaw_body",
    )
    top_jaw.visual(
        Box(size=(0.132, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, -0.016)),
        material=dark_rubber,
        name="jaw_pad",
    )
    top_jaw.visual(
        Box(size=(0.015, 0.010, 0.044)),
        origin=Origin(xyz=(-0.082, -0.029, -0.015)),
        material=satin_graphite,
        name="slide_tongue_0",
    )
    top_jaw.visual(
        Box(size=(0.015, 0.010, 0.044)),
        origin=Origin(xyz=(0.082, -0.029, -0.015)),
        material=satin_graphite,
        name="slide_tongue_1",
    )
    top_slide = model.articulation(
        "tilt_to_top_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt_cradle,
        child=top_jaw,
        origin=Origin(xyz=(0.0, 0.083, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.20, lower=0.0, upper=0.045),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.020,
            body_style="faceted",
            base_diameter=0.036,
            top_diameter=0.030,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0009, width=0.0018),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            top_feature=KnobTopFeature(style="top_insert", diameter=0.012, height=0.0010),
            center=False,
        ),
        "ribbed_control_knob",
    )

    pan_lock = model.part("pan_lock")
    pan_lock.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brushed_aluminum,
        name="threaded_stem",
    )
    pan_lock.visual(
        knob_mesh,
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=matte_black,
        name="knob_cap",
    )
    model.articulation(
        "pan_to_pan_lock",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=pan_lock,
        origin=Origin(xyz=(0.054, -0.030, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brushed_aluminum,
        name="axle_stub",
    )
    tilt_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=matte_black,
        name="knob_cap",
    )
    model.articulation(
        "pan_to_tilt_knob",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.079, 0.0, 0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=brushed_aluminum,
        name="clamp_screw",
    )
    clamp_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=matte_black,
        name="knob_cap",
    )
    model.articulation(
        "top_jaw_to_clamp_knob",
        ArticulationType.REVOLUTE,
        parent=top_jaw,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.025, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod")
    pan_head = object_model.get_part("pan_head")
    tilt_cradle = object_model.get_part("tilt_cradle")
    top_jaw = object_model.get_part("top_jaw")
    device = object_model.get_part("device")
    pan_lock = object_model.get_part("pan_lock")
    tilt_knob = object_model.get_part("tilt_knob")
    clamp_knob = object_model.get_part("clamp_knob")

    ctx.allow_overlap(
        pan_head,
        pan_lock,
        elem_a="pan_disk",
        elem_b="threaded_stem",
        reason="The pan lock stem is intentionally threaded a few millimeters into the pan disk boss.",
    )
    ctx.expect_gap(
        pan_lock,
        pan_head,
        axis="x",
        max_penetration=0.008,
        positive_elem="threaded_stem",
        negative_elem="pan_disk",
        name="pan lock stem seats locally in pan disk",
    )

    ctx.allow_overlap(
        top_jaw,
        clamp_knob,
        elem_a="jaw_body",
        elem_b="clamp_screw",
        reason="The clamp screw is intentionally represented as threaded into the sliding jaw body.",
    )
    ctx.expect_gap(
        clamp_knob,
        top_jaw,
        axis="y",
        max_penetration=0.008,
        positive_elem="clamp_screw",
        negative_elem="jaw_body",
        name="clamp screw remains locally seated in jaw",
    )

    ctx.allow_overlap(
        tilt_cradle,
        tilt_knob,
        elem_a="tilt_axle",
        elem_b="axle_stub",
        reason="The tilt knob stub is intentionally seated over the exposed tilt axle end.",
    )
    ctx.expect_gap(
        tilt_knob,
        tilt_cradle,
        axis="x",
        max_penetration=0.004,
        positive_elem="axle_stub",
        negative_elem="tilt_axle",
        name="tilt knob stub seats on axle end",
    )

    for idx in range(3):
        leg = object_model.get_part(f"leg_{idx}")
        ctx.allow_overlap(
            leg,
            tripod,
            elem_a="hinge_barrel",
            elem_b=f"leg_clevis_{idx}",
            reason="The leg hinge barrel is intentionally captured in the bored clevis at the tripod hub.",
        )
        ctx.expect_overlap(
            leg,
            tripod,
            axes="xy",
            elem_a="hinge_barrel",
            elem_b=f"leg_clevis_{idx}",
            min_overlap=0.010,
            name=f"leg {idx} hinge barrel is captured by clevis",
        )
        ctx.allow_overlap(
            leg,
            tripod,
            elem_a="hinge_tongue",
            elem_b=f"leg_clevis_{idx}",
            reason="The narrow hinge tongue passes through the clevis throat as the structural hinge lug.",
        )
        ctx.expect_overlap(
            leg,
            tripod,
            axes="z",
            elem_a="hinge_tongue",
            elem_b=f"leg_clevis_{idx}",
            min_overlap=0.010,
            name=f"leg {idx} hinge tongue passes through clevis throat",
        )

    for idx in (0, 1):
        ctx.allow_overlap(
            top_jaw,
            tilt_cradle,
            elem_a=f"slide_tongue_{idx}",
            elem_b=f"guide_rail_{idx}",
            reason=(
                "The sliding jaw tongues are intentionally represented as captured "
                "inside the bracket guide rails to show the adjustable clamp track."
            ),
        )
        ctx.expect_overlap(
            top_jaw,
            tilt_cradle,
            axes="z",
            elem_a=f"slide_tongue_{idx}",
            elem_b=f"guide_rail_{idx}",
            min_overlap=0.035,
            name=f"slide tongue {idx} remains retained in rail",
        )
        ctx.expect_within(
            top_jaw,
            tilt_cradle,
            axes="x",
            inner_elem=f"slide_tongue_{idx}",
            outer_elem=f"guide_rail_{idx}",
            margin=0.003,
            name=f"slide tongue {idx} stays centered in rail",
        )
        ctx.allow_overlap(
            top_jaw,
            tilt_cradle,
            elem_a=f"slide_tongue_{idx}",
            elem_b="bracket_backplate",
            reason="The sliding tongue is seated in a simplified solid guide channel in the backplate.",
        )

    ctx.check(
        "three articulated tripod legs",
        all(object_model.get_part(f"leg_{i}") is not None for i in range(3))
        and all(object_model.get_articulation(f"tripod_to_leg_{i}") is not None for i in range(3)),
        details="Expected three separate spread legs with hinge articulations at the hub.",
    )

    for i in range(3):
        aabb = ctx.part_element_world_aabb(object_model.get_part(f"leg_{i}"), elem="foot_pad")
        ctx.check(
            f"leg_{i} foot reaches floor",
            aabb is not None and aabb[0][2] <= 0.002 and aabb[1][2] >= 0.016,
            details=f"foot_pad_aabb={aabb}",
        )

    ctx.expect_gap(
        pan_head,
        tripod,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="pan_disk",
        negative_elem="pan_thrust_washer",
        name="pan disk seats on thrust washer",
    )
    ctx.expect_contact(
        tilt_cradle,
        pan_head,
        elem_a="tilt_axle",
        elem_b="tilt_yoke",
        contact_tol=0.004,
        name="tilt axle is captured by yoke",
    )
    ctx.expect_within(
        device,
        tilt_cradle,
        axes="x",
        margin=0.006,
        inner_elem="device_body",
        outer_elem="bracket_backplate",
        name="device is centered within bracket rails",
    )

    top_slide = object_model.get_articulation("tilt_to_top_jaw")
    rest_top = ctx.part_world_position(top_jaw)
    with ctx.pose({top_slide: 0.045}):
        raised_top = ctx.part_world_position(top_jaw)
        ctx.expect_gap(
            top_jaw,
            device,
            axis="z",
            min_gap=0.035,
            positive_elem="jaw_pad",
            negative_elem="device_body",
            name="sliding jaw opens above device",
        )
    ctx.check(
        "top jaw slides upward",
        rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.040,
        details=f"rest={rest_top}, raised={raised_top}",
    )

    tilt_joint = object_model.get_articulation("pan_to_tilt")
    rest_device = ctx.part_world_position(device)
    with ctx.pose({tilt_joint: 0.45}):
        tilted_device = ctx.part_world_position(device)
    ctx.check(
        "tilt pivot raises mounted device",
        rest_device is not None and tilted_device is not None and tilted_device[2] > rest_device[2] + 0.030,
        details=f"rest={rest_device}, tilted={tilted_device}",
    )

    return ctx.report()


object_model = build_object_model()
