from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _saddle_section(
    x: float,
    *,
    width: float,
    thickness: float,
    z_center: float,
    samples: int = 24,
) -> list[tuple[float, float, float]]:
    """Flattened oval section in the YZ plane for a shaped saddle shell."""
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        angle = tau * index / samples
        # A slightly squared-off, flatter underside reads more like a molded saddle shell.
        y = 0.5 * width * sin(angle)
        upper = max(cos(angle), 0.0)
        lower = min(cos(angle), 0.0)
        z = z_center + 0.54 * thickness * upper + 0.38 * thickness * lower
        points.append((x, y, z))
    return points


def _build_saddle_shell() -> MeshGeometry:
    sections = [
        _saddle_section(-0.138, width=0.122, thickness=0.026, z_center=0.112),
        _saddle_section(-0.095, width=0.148, thickness=0.034, z_center=0.108),
        _saddle_section(-0.030, width=0.132, thickness=0.032, z_center=0.104),
        _saddle_section(0.050, width=0.078, thickness=0.027, z_center=0.104),
        _saddle_section(0.115, width=0.044, thickness=0.023, z_center=0.108),
        _saddle_section(0.148, width=0.022, thickness=0.018, z_center=0.113),
    ]
    return repair_loft(section_loft(sections))


def _build_saddle_rail_loop() -> MeshGeometry:
    # One continuous stainless rail loop: two straight load rails joined by
    # visible front and rear bends under the shell.
    return tube_from_spline_points(
        [
            (-0.104, -0.026, 0.073),
            (-0.030, -0.026, 0.071),
            (0.086, -0.026, 0.073),
            (0.122, -0.010, 0.081),
            (0.122, 0.010, 0.081),
            (0.086, 0.026, 0.073),
            (-0.030, 0.026, 0.071),
            (-0.104, 0.026, 0.073),
            (-0.126, 0.010, 0.085),
            (-0.126, -0.010, 0.085),
        ],
        radius=0.003,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost_assembly")

    frame_paint = model.material("frame_paint", rgba=(0.05, 0.22, 0.25, 1.0))
    black_anodized = model.material("black_anodized", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.045, 0.048, 1.0))
    polished_black = model.material("polished_black", rgba=(0.02, 0.022, 0.026, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    bright_hardware = model.material("bright_hardware", rgba=(0.75, 0.74, 0.70, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.03, 0.03, 0.032, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.58, 0.42, 0.20, 1.0))

    seat_tube = model.part("seat_tube")
    seat_tube.visual(
        _mesh(
            "seat_tube_shell",
            _ring_shell(outer_radius=0.024, inner_radius=0.0172, z_min=0.0, z_max=0.320),
        ),
        material=frame_paint,
        name="seat_tube_shell",
    )
    seat_tube.visual(
        _mesh(
            "outer_sleeve",
            _ring_shell(outer_radius=0.0172, inner_radius=0.0144, z_min=0.120, z_max=0.548),
        ),
        material=black_anodized,
        name="outer_sleeve",
    )
    seat_tube.visual(
        _mesh(
            "clamp_collar",
            _ring_shell(outer_radius=0.029, inner_radius=0.0171, z_min=0.306, z_max=0.345),
        ),
        material=black_anodized,
        name="clamp_collar",
    )
    seat_tube.visual(
        _mesh(
            "wiper_seal",
            _ring_shell(outer_radius=0.0205, inner_radius=0.0143, z_min=0.532, z_max=0.560),
        ),
        material=seal_rubber,
        name="wiper_seal",
    )
    seat_tube.visual(
        _mesh(
            "guide_bushing",
            _ring_shell(outer_radius=0.0144, inner_radius=0.01315, z_min=0.492, z_max=0.532, segments=96),
        ),
        material=bearing_bronze,
        name="guide_bushing",
    )
    # Seat clamp split, pinch ears, and bolt hardware.
    seat_tube.visual(
        Box((0.006, 0.004, 0.067)),
        origin=Origin(xyz=(0.0, 0.0245, 0.330)),
        material=satin_black,
        name="collar_split",
    )
    for x in (-0.010, 0.010):
        seat_tube.visual(
            Box((0.010, 0.014, 0.030)),
            origin=Origin(xyz=(x, 0.032, 0.326)),
            material=black_anodized,
            name=f"clamp_ear_{0 if x < 0 else 1}",
        )
    seat_tube.visual(
        Cylinder(radius=0.0032, length=0.040),
        origin=Origin(xyz=(0.0, 0.041, 0.326), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_hardware,
        name="collar_bolt",
    )
    seat_tube.visual(
        Cylinder(radius=0.0058, length=0.004),
        origin=Origin(xyz=(-0.022, 0.041, 0.326), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_hardware,
        name="collar_bolt_head",
    )
    seat_tube.visual(
        Cylinder(radius=0.0058, length=0.004),
        origin=Origin(xyz=(0.022, 0.041, 0.326), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_hardware,
        name="collar_nut",
    )

    post_body = model.part("post_body")
    post_body.visual(
        Cylinder(radius=0.0132, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=polished_black,
        name="stanchion",
    )
    post_body.visual(
        Cylinder(radius=0.0170, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=black_anodized,
        name="top_cap",
    )
    for z in (0.198, 0.236, 0.274):
        post_body.visual(
            Cylinder(radius=0.0137, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bright_hardware,
            name=f"height_mark_{int(z * 1000)}",
        )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.0178, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=black_anodized,
        name="post_socket",
    )
    saddle_clamp.visual(
        Box((0.086, 0.058, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0415)),
        material=black_anodized,
        name="cradle_bridge",
    )
    for y in (-0.034, 0.034):
        index = 0 if y < 0 else 1
        saddle_clamp.visual(
            Box((0.070, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.050)),
            material=black_anodized,
            name=f"bolt_lug_{index}",
        )
    for y in (-0.026, 0.026):
        index = 0 if y < 0 else 1
        cradle_name = "lower_cradle_0" if y < 0 else "lower_cradle_1"
        saddle_clamp.visual(
            Box((0.086, 0.006, 0.015)),
            origin=Origin(xyz=(0.0, y, 0.0545)),
            material=black_anodized,
            name=f"cradle_web_{index}",
        )
        saddle_clamp.visual(
            Cylinder(radius=0.0040, length=0.092),
            origin=Origin(xyz=(0.0, y, 0.066), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_anodized,
            name=cradle_name,
        )
    for x in (-0.026, 0.026):
        index = 0 if x < 0 else 1
        saddle_clamp.visual(
            Box((0.024, 0.076, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.081)),
            material=black_anodized,
            name=f"upper_plate_{index}",
        )
    for x in (-0.026, 0.026):
        for y in (-0.034, 0.034):
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            saddle_clamp.visual(
                Cylinder(radius=0.0026, length=0.046),
                origin=Origin(xyz=(x, y, 0.066)),
                material=bright_hardware,
                name=f"clamp_bolt_{suffix}",
            )
            saddle_clamp.visual(
                Cylinder(radius=0.0053, length=0.0035),
                origin=Origin(xyz=(x, y, 0.0905)),
                material=bright_hardware,
                name=f"bolt_head_{suffix}",
            )

    saddle = model.part("saddle")
    saddle.visual(
        _mesh("saddle_shell", _build_saddle_shell()),
        material=saddle_vinyl,
        name="saddle_shell",
    )
    saddle.visual(
        _mesh("rail_loop", _build_saddle_rail_loop()),
        material=rail_steel,
        name="rail_loop",
    )
    for x in (-0.096, 0.096):
        for y in (-0.026, 0.026):
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            saddle.visual(
                Cylinder(radius=0.0045, length=0.026),
                origin=Origin(xyz=(x, y, 0.088)),
                material=satin_black,
                name=f"rail_boss_{suffix}",
            )
    saddle.visual(
        Box((0.045, 0.080, 0.006)),
        origin=Origin(xyz=(-0.106, 0.0, 0.094)),
        material=satin_black,
        name="rear_rail_plate",
    )
    saddle.visual(
        Box((0.036, 0.044, 0.006)),
        origin=Origin(xyz=(0.106, 0.0, 0.096)),
        material=satin_black,
        name="front_rail_plate",
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=seat_tube,
        child=post_body,
        origin=Origin(xyz=(0.0, 0.0, 0.548)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    model.articulation(
        "post_to_clamp",
        ArticulationType.FIXED,
        parent=post_body,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=saddle_clamp,
        child=saddle,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    seat_tube = object_model.get_part("seat_tube")
    post_body = object_model.get_part("post_body")
    saddle_clamp = object_model.get_part("saddle_clamp")
    saddle = object_model.get_part("saddle")
    seat_height = object_model.get_articulation("seat_height")

    ctx.allow_overlap(
        seat_tube,
        post_body,
        elem_a="guide_bushing",
        elem_b="stanchion",
        reason="The top guide bushing is modeled as a very small sliding interference fit that supports the dropper stanchion.",
    )
    ctx.expect_within(
        post_body,
        seat_tube,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="stanchion stays centered in sleeve",
    )
    ctx.expect_overlap(
        post_body,
        seat_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="collapsed post remains deeply inserted",
    )
    ctx.expect_overlap(
        post_body,
        seat_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="guide_bushing",
        min_overlap=0.035,
        name="stanchion passes through guide bushing",
    )
    ctx.expect_contact(
        saddle,
        saddle_clamp,
        elem_a="rail_loop",
        elem_b="lower_cradle_0",
        contact_tol=0.003,
        name="saddle rail rests on clamp cradle",
    )
    ctx.expect_overlap(
        saddle,
        saddle_clamp,
        axes="xy",
        elem_a="rail_loop",
        elem_b="cradle_bridge",
        min_overlap=0.040,
        name="rail loop passes through clamp footprint",
    )

    rest_pos = ctx.part_world_position(post_body)
    with ctx.pose({seat_height: 0.180}):
        ctx.expect_within(
            post_body,
            seat_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="extended stanchion stays coaxial",
        )
        ctx.expect_overlap(
            post_body,
            seat_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_sleeve",
            min_overlap=0.065,
            name="extended post keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(post_body)
    ctx.check(
        "seat height extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.17,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
